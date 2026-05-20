"""
simulation.py
Ana simülasyon döngüsü.

Bir adımda:
1. Robot kinematik kontrolü (hedefe doğru basit kontrolcü)
2. LiDAR tarama
3. IMU + Enkoder ölçümü
4. EKF predict + update (sensör füzyonu)
5. Dead reckoning güncelleme
6. Pygame çizim
7. Hedef kontrolü
"""

import numpy as np
import sys
import os

# Proje kökünü path'e ekle
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from environment.map import GridMap
from robots import get_robot, BaseRobot
from sensors import LiDAR, IMU, WheelEncoder
from localization import EKF, DeadReckoning
from planners import get_planner, LOCAL_PLANNER_TYPES, get_local_planner
from visualization import PyGameRenderer


class Simulation:
    """
    Tüm bileşenleri birleştiren simülasyon yöneticisi.

    Args:
        robot_type:   'differential' | 'omni' | 'ackermann' | 'mecanum'
        planner_type: 'astar' | 'dijkstra' | 'dstar' | 'rrt' | 'rrt*'
        metric:       'euclidean' | 'manhattan' | 'chebyshev' | 'octile' | ...
        criteria:     'shortest' | 'safest' | 'fastest' | 'smoothest'
        padding:      Engel şişirme hücre yarıçapı (0 = kapalı).
        dt:           Simülasyon zaman adımı (s).
        cell_size:    Harita hücresi gerçek boyutu (m).
    """

    def __init__(self, robot_type: str = "differential",
                 planner_type: str = "astar",
                 metric: str = "euclidean",
                 criteria: str = "shortest",
                 padding: int = 1,
                 dt: float = 0.05,
                 cell_size: float = 0.5,
                 headless: bool = False,
                 output_tag: str = "",
                 screen=None,
                 sensor_params: dict = None):

        self.dt          = dt
        self._headless   = headless
        self._output_tag = output_tag   # kayıt dosya eki ("astar_euclidean" vb.)

        # Harita
        self.grid = GridMap(rows=50, cols=50, cell_size=cell_size)

        # Engel şişirme (robot robot_radius / cell_size kadar)
        if padding > 0:
            self.grid.inflate_obstacles(padding_cells=padding)
            print(f"Engel dolgusu: {padding} hücre "
                  f"({padding * cell_size:.2f} m)")

        # Clearance haritası (safest kriteri için)
        cost_map = None
        if criteria == "safest":
            print("Clearance haritası hesaplanıyor...")
            cost_map = self.grid.clearance_map()

        # Başlangıç / hedef (dünya koordinatı)
        sr, sc = self.grid.start
        gr, gc = self.grid.goal
        x0 = (sc + 0.5) * cell_size
        y0 = (sr + 0.5) * cell_size
        self.goal_world = ((gc + 0.5) * cell_size, (gr + 0.5) * cell_size)

        # Robot
        self.robot: BaseRobot = get_robot(robot_type, x=x0, y=y0,
                                          theta=0.0)
        # Canlı ayar paneli için başlangıç değerleri saklanıyor
        self._init_robot_type   = robot_type
        self._init_planner_type = planner_type
        self._init_metric       = metric
        self._init_criteria     = criteria

        # Sensörler
        _sp = sensor_params or {}
        self.lidar   = LiDAR(
            num_beams  = int(_sp.get("lidar_beams", 180)),
            max_range  = float(_sp.get("lidar_range", 8.0)),
            noise_std  = float(_sp.get("lidar_noise", 0.04)),
            resolution = cell_size * 0.5)
        self.imu     = IMU(gyro_noise_std=0.008, gyro_bias=0.003)
        self.encoder = WheelEncoder(ticks_per_rev=500,
                                    wheel_radius=0.1,
                                    wheel_base=0.5)

        # Lokalizasyon
        self.ekf = EKF(x0=x0, y0=y0, theta0=0.0)
        self.dr  = DeadReckoning(x0=x0, y0=y0, theta0=0.0)

        # ── Planlayıcı: Global mi Local mi? ────────────────────────────────────
        _ptype_key = planner_type.lower().strip()
        self._local_mode = _ptype_key in LOCAL_PLANNER_TYPES

        if self._local_mode:
            # ── Local (reaktif) planlayıcı — global yol hesaplanmaz ──────────
            self._local_planner = get_local_planner(planner_type)
            self._local_planner.reset(x0, y0, *self.goal_world)
            self.path_grid  = []
            self.path_world = []
            self._wp_idx    = 0
            self.path_stats = {
                'planner':        self._local_planner.name,
                'metric':         'local',
                'criteria':       'reaktif',
                'path_length':    0,
                'nodes_expanded': 0,
            }
        else:
            # ── Global planlayıcı — tam yol önceden hesaplanır ───────────────
            self._local_planner = None
            planner = get_planner(planner_type, self.grid, metric=metric,
                                  criteria=criteria, cost_map=cost_map)
            self.path_grid  = planner.plan(self.grid.start, self.grid.goal)
            self.path_stats = planner.stats()

            if not self.path_grid:
                raise RuntimeError("Yol bulunamadı! Haritayı veya planlayıcıyı kontrol et.")

            from planners import smooth_path
            self.path_world = smooth_path(self.path_grid, self.grid)
            self._wp_idx    = 0

        # Waypoint indexi (global modda kullanılır; local modda sıfır kalır)
        self._wp_idx = 0

        # Pygame renderer (headless modda None)
        if not headless:
            self.renderer = PyGameRenderer(self.grid, cell_px=14, fps=30,
                                           screen=screen,
                                           init_settings={
                                               "robot":    robot_type,
                                               "planner":  planner_type,
                                               "metric":   metric,
                                               "criteria": criteria,
                                           })
            self.renderer.set_robot_type(self.robot.robot_type)
            self.renderer.set_planned_path(self.path_grid)
            self.renderer.set_smooth_path(self.path_world)
        else:
            self.renderer = None

        # Hata kaydı
        self.position_errors: list = []
        self.timestamps:      list = []
        self._t = 0.0
        self._lidar_snapshots: list = []

        print(f"\n{'='*50}")
        print(f"Robot      : {self.robot.robot_type}")
        print(f"Planlayıcı : {self.path_stats['planner']}", end="")
        if self._local_mode:
            print("  (Local / Reaktif LiDAR)")
        else:
            print()
            print(f"Metrik     : {self.path_stats['metric']}")
            print(f"Kriter     : {self.path_stats['criteria']}")
            print(f"Yol uzunl. : {self.path_stats['path_length']} hücre")
            print(f"Genişletil.: {self.path_stats['nodes_expanded']} düğüm")
        print(f"{'='*50}\n")
        print("SPACE: duraklat | +/-: hız | ESC: çıkış\n")

    # ------------------------------------------------------------------
    # Ana döngü
    # ------------------------------------------------------------------

    def run(self):
        if self._headless:
            return self._run_headless()
        return self._run_interactive()

    def _run_headless(self):
        """Pygame olmadan maksimum hızda koşar, sadece çıktı kaydeder."""
        MAX_STEPS = 20000
        prev_enc  = None
        step      = 0

        while step < MAX_STEPS:
            step += 1
            # Local modda LiDAR taraması kontrol öncesinde yapılır
            if self._local_mode:
                self.lidar.scan(
                    self.robot.state.x, self.robot.state.y,
                    self.robot.state.theta, self.grid
                )
            # LiDAR snapshot (görselleştirme için, 3 farklı anında)
            if step in (1, 200, 600) and len(self._lidar_snapshots) < 3:
                if not self._local_mode:
                    self.lidar.scan(self.robot.state.x, self.robot.state.y,
                                    self.robot.state.theta, self.grid)
                self._lidar_snapshots.append({
                    'raw':  self.lidar.to_point_cloud(
                                self.robot.state.x, self.robot.state.y,
                                self.robot.state.theta, use_filtered=False),
                    'filt': self.lidar.to_point_cloud(
                                self.robot.state.x, self.robot.state.y,
                                self.robot.state.theta, use_filtered=True),
                    'pose': (self.robot.state.x, self.robot.state.y,
                             self.robot.state.theta),
                })
            ctrl = self._compute_control()
            self.robot.kinematic_step(ctrl, self.dt)

            # Sensörler
            true_v = np.hypot(self.robot.state.vx, self.robot.state.vy)
            imu_m  = self.imu.measure(self.robot.state.omega, 0.0, 0.0, self.dt)

            if hasattr(self.robot, 'wheel_speeds') and \
               self.robot.robot_type == "DifferentialDrive":
                vl, vr = self.robot.wheel_speeds(true_v, self.robot.state.omega)
            else:
                vl = vr = true_v / max(getattr(self.robot, 'r', 0.1), 0.01)

            enc_m = self.encoder.measure(vl, vr, self.dt)
            v_enc = true_v
            if prev_enc is not None:
                v_enc, _ = self.encoder.odometry(enc_m, prev_enc, self.dt)
            prev_enc = enc_m

            LIDAR_POS_STD = 0.20
            lidar_xy = (
                self.robot.state.x + np.random.normal(0, LIDAR_POS_STD),
                self.robot.state.y + np.random.normal(0, LIDAR_POS_STD),
            )
            self.ekf.fuse(v_enc, imu_m.omega_measured, self.dt,
                          lidar_xy=lidar_xy)
            self.dr.update(v_enc, imu_m.omega_measured, self.dt)

            self._t += self.dt
            ex = self.robot.state.x - self.ekf.x[0]
            ey = self.robot.state.y - self.ekf.x[1]
            self.position_errors.append(np.hypot(ex, ey))
            self.timestamps.append(self._t)

            gx, gy = self.goal_world
            if np.hypot(self.robot.state.x - gx,
                        self.robot.state.y - gy) < self.grid.cell_size * 1.5:
                break

        self._save_outputs()
        return True

    def _run_interactive(self):
        prev_enc = None

        while True:
            # --- 1. LiDAR (local modda kontrol için; her modda görselleştirme için) ---
            raw_r, filt_r = self.lidar.scan(
                self.robot.state.x, self.robot.state.y,
                self.robot.state.theta, self.grid
            )
            raw_cloud  = self.lidar.to_point_cloud(
                self.robot.state.x, self.robot.state.y,
                self.robot.state.theta, use_filtered=False)
            filt_cloud = self.lidar.to_point_cloud(
                self.robot.state.x, self.robot.state.y,
                self.robot.state.theta, use_filtered=True)

            # --- 2. Kontrol ---
            ctrl = self._compute_control()
            self.robot.kinematic_step(ctrl, self.dt)

            # --- 3. IMU + Enkoder ---
            true_v = np.hypot(self.robot.state.vx, self.robot.state.vy)
            imu_m  = self.imu.measure(self.robot.state.omega,
                                      0.0, 0.0, self.dt)

            # Diferansiyel için tekerlek hızları
            if hasattr(self.robot, 'wheel_speeds') and \
               self.robot.robot_type == "DifferentialDrive":
                vl, vr = self.robot.wheel_speeds(true_v, self.robot.state.omega)
            else:
                vl = vr = true_v / max(getattr(self.robot, 'r', 0.1), 0.01)

            enc_m = self.encoder.measure(vl, vr, self.dt)
            if prev_enc is not None:
                v_enc, _ = self.encoder.odometry(enc_m, prev_enc, self.dt)
            else:
                v_enc = true_v
            prev_enc = enc_m

            # --- 4. EKF ---
            obs = self.lidar.detect_obstacles(
                self.robot.state.x, self.robot.state.y,
                self.robot.state.theta)
            # LiDAR konum ölçümü simülasyonu:
            # Gerçek sistemde scan-matching ile elde edilir.
            # Burada doğru konuma Gaussian gürültü ekleyerek simüle ediyoruz.
            LIDAR_POS_STD = 0.20  # metre — LiDAR konum ölçüm gürültüsü
            lidar_xy = (
                self.robot.state.x + np.random.normal(0, LIDAR_POS_STD),
                self.robot.state.y + np.random.normal(0, LIDAR_POS_STD),
            )
            self.ekf.fuse(v_enc, imu_m.omega_measured, self.dt,
                          lidar_xy=lidar_xy)

            # --- 5. Dead reckoning ---
            self.dr.update(v_enc, imu_m.omega_measured, self.dt)

            # --- 6. Hata kaydı ---
            self._t += self.dt
            ex = self.robot.state.x - self.ekf.x[0]
            ey = self.robot.state.y - self.ekf.x[1]
            self.position_errors.append(np.hypot(ex, ey))
            self.timestamps.append(self._t)

            # --- 7. Render ---
            hud = {
                "Planlayıcı": self.path_stats['planner'],
                "Metrik":     self.path_stats['metric'],
                "Kriter":     self.path_stats['criteria'],
                "EKF err":    f"{self.position_errors[-1]:.3f} m",
                "Süre":       f"{self._t:.1f} s",
            }

            # ICR hesabı (sadece diferansiyel için)
            icr_data = None
            if hasattr(self.robot, 'icr'):
                icr_data = self.robot.icr()

            action = self.renderer.update(
                self.robot,
                raw_cloud, filt_cloud,
                ekf_pose=self.ekf.pose(),
                dr_pose=self.dr.pose(),
                hud_extra=hud,
                wp_idx=self._wp_idx,
                icr=icr_data
            )

            if action in ("replay", "menu", "quit"):
                self.renderer.close()
                self._save_outputs()
                if action == "replay":
                    live = self.renderer._live.copy()
                    return {
                        "action":   "replay",
                        "robot":    live.get("robot",    self._init_robot_type),
                        "planner":  live.get("planner",  self._init_planner_type),
                        "metric":   live.get("metric",   self._init_metric),
                        "criteria": live.get("criteria", self._init_criteria),
                    }
                return action

            if action == "paused":
                while True:
                    act2 = self.renderer.handle_paused()
                    if act2 != "paused":
                        break
                if act2 in ("replay", "menu", "quit"):
                    self.renderer.close()
                    self._save_outputs()
                    return act2

            # --- 8. Hedef kontrolü ---
            gx, gy = self.goal_world
            dist_to_goal = np.hypot(
                self.robot.state.x - gx,
                self.robot.state.y - gy
            )
            if dist_to_goal < self.grid.cell_size * 1.5:
                print(f"\nHedefe ulaşıldı! Süre: {self._t:.1f}s")
                self._print_stats()
                action = self.renderer.show_arrived(self._t, self.position_errors)
                self.renderer.close()
                self._save_outputs()
                if action == "replay":
                    live = self.renderer._live.copy()
                    return {
                        "action":   "replay",
                        "robot":    live.get("robot",    self._init_robot_type),
                        "planner":  live.get("planner",  self._init_planner_type),
                        "metric":   live.get("metric",   self._init_metric),
                        "criteria": live.get("criteria", self._init_criteria),
                    }
                return action

    # ------------------------------------------------------------------
    # Local planlayıcı kontrolcüsü
    # ------------------------------------------------------------------

    def _compute_control_local(self):
        """
        Local (reaktif) planlayıcı için kontrol komutu hesapla.
        LiDAR filtered_ranges + hedef → (v, omega) → robot-özgü komut.
        """
        gx, gy = self.goal_world
        # self.lidar.filtered_ranges: mevcut adımda scan() ile güncellendi
        v, omega = self._local_planner.compute(
            self.robot.state.x,
            self.robot.state.y,
            self.robot.state.theta,
            self.lidar.filtered_ranges,
            self.lidar.beam_angles,
            gx, gy
        )
        return self._local_to_ctrl(v, omega)

    def _local_to_ctrl(self, v: float, omega: float):
        """
        Local planlayıcının (v, omega) çıktısını robota özgü kontrol sinyaline çevir.
        Global modda _compute_control ile aynı dönüşüm mantığını kullanır.
        """
        import math
        rtype = self.robot.robot_type

        # Robot limitlerinde kırp
        v     = float(np.clip(v,     0.0, self.robot.max_linear_speed  * 0.75))
        omega = float(np.clip(omega, -self.robot.max_angular_speed,
                                      self.robot.max_angular_speed))

        if rtype == "DifferentialDrive":
            return (v, omega)
        elif rtype == "OmniWheel":
            return (v, 0.0, omega)
        elif rtype == "Ackermann":
            v_safe = max(v, self.robot.max_linear_speed * 0.18)
            delta  = np.clip(
                math.atan2(omega * self.robot.L, v_safe),
                -self.robot.delta_max, self.robot.delta_max
            )
            return (v_safe, delta)
        elif rtype in ("Mecanum", "Quadruped", "Hexapod"):
            return (v, 0.0, omega)
        elif rtype in ("Quadrotor", "VTOL"):
            vx = v * np.cos(self.robot.state.theta)
            vy = v * np.sin(self.robot.state.theta)
            return (vx, vy, omega)
        elif rtype == "FixedWing":
            v_fw  = max(v, self.robot.max_linear_speed * 0.3)
            delta = np.clip(
                math.atan2(omega * self.robot.L, v_fw),
                -self.robot.bank_max, self.robot.bank_max
            )
            return (v_fw, delta)
        # SnakeRobot, Bipedal ve bilinmeyen → (v, omega)
        return (v, omega)

    # ------------------------------------------------------------------
    # Global planlayıcı kontrolcüsü (Waypoint takibi — Pure Pursuit benzeri)
    # ------------------------------------------------------------------

    def _compute_control(self):
        """
        Güncel waypoint'e doğru basit açı-hız kontrolcüsü.
        Tüm robot tiplerine ortak basit kontrol: (v, omega) veya eşdeğeri.
        Local modda local planlayıcıya delege edilir.
        """
        if self._local_mode:
            return self._compute_control_local()

        if self._wp_idx >= len(self.path_world):
            return self._zero_ctrl()

        rtype = self.robot.robot_type

        # ── FixedWing: Pure Pursuit lookahead ────────────────────────
        # Minimum dönüş yarıçapından daha yakın waypoint'leri atla
        if rtype == "FixedWing":
            min_look = max(self.robot.min_turn_radius(), self.grid.cell_size * 2.0)
            while self._wp_idx < len(self.path_world) - 1:
                wx_t, wy_t = self.path_world[self._wp_idx]
                if np.hypot(wx_t - self.robot.state.x,
                            wy_t - self.robot.state.y) >= min_look:
                    break
                self._wp_idx += 1

        # Waypoint dünya koordinatı
        wx, wy = self.path_world[self._wp_idx]

        dx = wx - self.robot.state.x
        dy = wy - self.robot.state.y
        dist = np.hypot(dx, dy)

        # Waypoint'e yeterince yaklaştıysa ilerle
        wp_thresh = self.grid.cell_size * (1.5 if rtype == "FixedWing" else 0.35)
        if dist < wp_thresh:
            self._wp_idx += 1
            if self._wp_idx >= len(self.path_world):
                return self._zero_ctrl()
            wx, wy = self.path_world[self._wp_idx]
            dx = wx - self.robot.state.x
            dy = wy - self.robot.state.y
            dist = np.hypot(dx, dy)

        desired_theta = np.arctan2(dy, dx)
        theta_err = desired_theta - self.robot.state.theta
        theta_err = (theta_err + np.pi) % (2 * np.pi) - np.pi

        v     = np.clip(0.8 * dist, 0, self.robot.max_linear_speed * 0.6)
        omega = np.clip(2.0 * theta_err, -self.robot.max_angular_speed,
                        self.robot.max_angular_speed)

        if rtype == "DifferentialDrive":
            return (v, omega)
        elif rtype == "OmniWheel":
            return (v * np.cos(theta_err), v * np.sin(theta_err), omega)
        elif rtype == "Ackermann":
            import math
            delta = np.clip(math.atan2(omega * self.robot.L, v),
                            -self.robot.delta_max, self.robot.delta_max)
            return (v, delta)
        elif rtype in ("Mecanum", "Quadruped", "Hexapod"):
            # Gövde çerçevesi — kinematic_step içinde dünya çerçevesine dönüştürülür
            return (v * np.cos(theta_err), v * np.sin(theta_err), omega)
        elif rtype in ("Quadrotor", "VTOL"):
            if dist > 1e-6:
                return (v * dx / dist, v * dy / dist, omega)
            return (0.0, 0.0, 0.0)
        elif rtype == "FixedWing":
            import math
            v_fw = max(v, self.robot.max_linear_speed * 0.3)
            delta = np.clip(math.atan2(omega * self.robot.L, v_fw),
                            -self.robot.bank_max, self.robot.bank_max)
            return (v_fw, delta)
        # SnakeRobot, Bipedal ve bilinmeyen → diferansiyel benzeri
        return (v, omega)

    def _zero_ctrl(self):
        rtype = self.robot.robot_type
        if rtype in ("OmniWheel", "Mecanum", "Quadrotor", "VTOL",
                      "Quadruped", "Hexapod"):
            return (0.0, 0.0, 0.0)
        if rtype == "FixedWing":
            return (self.robot.max_linear_speed * 0.3, 0.0)
        return (0.0, 0.0)

    # ------------------------------------------------------------------
    # Sonuç ve kayıt
    # ------------------------------------------------------------------

    def _print_stats(self):
        import numpy as np
        errs = np.array(self.position_errors)
        rmse = np.sqrt(np.mean(errs ** 2))
        mae  = np.mean(np.abs(errs))
        print(f"  RMSE: {rmse:.4f} m")
        print(f"  MAE:  {mae:.4f} m")
        print(f"  Max hata: {errs.max():.4f} m")

    def _save_outputs(self):
        """Matplotlib ile çıktı grafiklerini kombinasyona özel alt klasöre kaydet."""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            import numpy as np

            # ── Çıktı klasörü: outputs/<robot>__<planner>__<metric>__<criteria>/ ──
            rtype    = self.robot.robot_type.lower()
            planner  = self.path_stats.get('planner',  'unknown').lower()
            metric   = self.path_stats.get('metric',   'unknown').lower()
            criteria = self.path_stats.get('criteria', 'unknown').lower()
            # Görünen isimleri dosya adı uyumlu isimlere çevir
            _norm = {"a*": "astar", "rrt*": "rrtstar", "d*": "dstar",
                     "d* lite": "dstar", "differentialdrive": "diff",
                     "omniwheel": "omni", "quadruped": "quad",
                     "hexapod": "hex", "snakerobot": "snake",
                     "bipedal": "biped"}
            rtype    = _norm.get(rtype,    rtype)
            planner  = _norm.get(planner,  planner)
            # Kalan geçersiz karakterleri temizle (* ? : < > | / \)
            def _safe(s):
                for ch in '*?:<>|\\/. ':
                    s = s.replace(ch, '')
                return s
            folder_name = f"{_safe(rtype)}__{_safe(planner)}__{_safe(metric)}__{_safe(criteria)}"
            if self._output_tag:
                out_dir = os.path.join("outputs", self._output_tag, folder_name)
            else:
                out_dir = os.path.join("outputs", folder_name)
            os.makedirs(out_dir, exist_ok=True)

            # ── 1. Hata grafiği ──────────────────────────────────────
            fig, ax = plt.subplots(figsize=(10, 4))
            fig.patch.set_facecolor("#0a0f1c")
            ax.set_facecolor("#0a0f1c")
            errs_ekf = np.array(self.position_errors)
            t_arr    = np.array(self.timestamps[:len(errs_ekf)])
            ax.plot(t_arr, errs_ekf, color='#00C8DC', lw=1.5, label='EKF konum hatası')
            _dr_h = self.dr.history; _tr_h = self.robot.history
            _n_dr = min(len(_tr_h), len(_dr_h), len(t_arr))
            if _n_dr > 0:
                _errs_dr = np.array([
                    np.hypot(_tr_h[i][0] - _dr_h[i][0], _tr_h[i][1] - _dr_h[i][1])
                    for i in range(_n_dr)])
                ax.plot(t_arr[:_n_dr], _errs_dr, '--', color='#FFD700', lw=1.2,
                        label='Dead Reckoning hatası')
            ax.set_title(
                f"Konum Hatası — {self.robot.robot_type} / "
                f"{self.path_stats['planner']} / {self.path_stats['criteria']}",
                color='#88CCFF')
            ax.set_xlabel("Zaman (s)", color='#AAC')
            ax.set_ylabel("Hata (m)", color='#AAC')
            ax.tick_params(colors='#AAC')
            for sp in ax.spines.values(): sp.set_color('#445')
            ax.grid(True, alpha=0.2, color='#335')
            if len(errs_ekf):
                _rmse = float(np.sqrt(np.mean(errs_ekf**2)))
                _mae  = float(np.mean(np.abs(errs_ekf)))
                ax.axhline(_rmse, color='#00C8DC', linestyle=':',
                           alpha=0.6, label=f"EKF RMSE={_rmse:.3f}m  MAE={_mae:.3f}m")
            ax.legend(facecolor='#111', labelcolor='white', fontsize=9)
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, "hata.png"), dpi=150)
            plt.close(fig)

            # ── 2. Yol haritası ──────────────────────────────────────
            fig, ax = plt.subplots(figsize=(8, 8))
            ax.set_facecolor("#0a0f1c")
            fig.patch.set_facecolor("#0a0f1c")

            # Engel + padding hücreleri
            for r in range(self.grid.rows):
                for c in range(self.grid.cols):
                    if self.grid.is_padding(r, c):
                        ax.add_patch(plt.Rectangle(
                            (c - 0.5, r - 0.5), 1, 1,
                            color='#8B5A2B', alpha=0.6))
                    elif not self.grid.is_free(r, c):
                        ax.add_patch(plt.Rectangle(
                            (c - 0.5, r - 0.5), 1, 1,
                            color='#505560'))

            # Planlanan yol
            if self.path_grid:
                pr = [p[1] for p in self.path_grid]
                pc = [p[0] for p in self.path_grid]
                ax.plot(pr, pc, ':', color='#88AAFF',
                        lw=1.5, label='Planlanan yol')

            # Gerçek yol
            if self.robot.history:
                hx = [h[0] / self.grid.cell_size for h in self.robot.history]
                hy = [h[1] / self.grid.cell_size for h in self.robot.history]
                ax.plot(hx, hy, '-', color='#32C850',
                        lw=2, label='Gerçek yol')

            # EKF tahmini
            if self.ekf.history:
                ex2 = [h[0] / self.grid.cell_size for h in self.ekf.history]
                ey2 = [h[1] / self.grid.cell_size for h in self.ekf.history]
                ax.plot(ex2, ey2, '--', color='#00C8DC',
                        lw=1.5, label='EKF tahmini')

            # Dead Reckoning tahmini
            if self.dr.history:
                _dx2 = [h[0] / self.grid.cell_size for h in self.dr.history]
                _dy2 = [h[1] / self.grid.cell_size for h in self.dr.history]
                ax.plot(_dx2, _dy2, ':', color='#FFD700',
                        lw=1.2, label='Dead Reckoning', alpha=0.8)

            # Başlangıç / hedef
            sr, sc = self.grid.start
            gr, gc = self.grid.goal
            ax.scatter([sc], [sr], c='lime',   s=150, zorder=5,
                       marker='*', label='Başlangıç')
            ax.scatter([gc], [gr], c='tomato', s=150, zorder=5,
                       marker='X', label='Hedef')

            ax.set_xlim(0, self.grid.cols)
            ax.set_ylim(0, self.grid.rows)
            ax.set_xlabel("Sütun", color='#AAC')
            ax.set_ylabel("Satır",  color='#AAC')
            ax.tick_params(colors='#AAC')
            for sp in ax.spines.values():
                sp.set_color('#445')
            ax.set_title(
                f"Yol Haritası — {self.robot.robot_type} | "
                f"{self.path_stats['planner']} | {self.path_stats['criteria']}",
                color='#88CCFF')
            ax.legend(loc='upper right', fontsize=8,
                      facecolor='#111', labelcolor='white')
            ax.grid(True, alpha=0.15, color='#335')
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, "yol.png"), dpi=150)
            plt.close(fig)

            # ── 3. LiDAR görselleştirmesi ──────────────────────────
            if self._lidar_snapshots:
                _snap = self._lidar_snapshots[len(self._lidar_snapshots) // 2]
                _fig2, _ax2s = plt.subplots(1, 2, figsize=(12, 6))
                _fig2.patch.set_facecolor("#0a0f1c")
                _fig2.suptitle(
                    f"LiDAR Sensör Görselleştirmesi — {self.robot.robot_type}"
                    f" | {self.path_stats['planner']}",
                    color='#88CCFF', fontsize=12)
                _cs2 = self.grid.cell_size
                for _ax2, _pts, _ttl, _col in [
                    (_ax2s[0], _snap['raw'],  'Ham LiDAR Verisi',          '#FF5555'),
                    (_ax2s[1], _snap['filt'], 'Filtrelenmiş LiDAR Verisi', '#44FF88'),
                ]:
                    _ax2.set_facecolor("#0a0f1c")
                    for _r in range(self.grid.rows):
                        for _c in range(self.grid.cols):
                            if not self.grid.is_free(_r, _c):
                                _ax2.add_patch(plt.Rectangle(
                                    (_c * _cs2, _r * _cs2), _cs2, _cs2,
                                    color='#505560', alpha=0.6, zorder=1))
                    _rx0, _ry0 = _snap['pose'][0], _snap['pose'][1]
                    _ax2.scatter([_rx0], [_ry0], c='white', s=80, zorder=5,
                                 marker='^', label='Robot')
                    _pts_arr = np.asarray(_pts)
                    if len(_pts_arr) > 0:
                        _ax2.scatter(_pts_arr[:, 0], _pts_arr[:, 1], c=_col, s=4,
                                     alpha=0.7, zorder=3, label=_ttl)
                    _ax2.set_xlim(0, self.grid.cols * _cs2)
                    _ax2.set_ylim(0, self.grid.rows * _cs2)
                    _ax2.set_title(_ttl, color='#88CCFF', fontsize=10)
                    _ax2.set_xlabel("X (m)", color='#AAC')
                    _ax2.set_ylabel("Y (m)", color='#AAC')
                    _ax2.tick_params(colors='#AAC')
                    for _sp in _ax2.spines.values(): _sp.set_color('#445')
                    _ax2.legend(loc='upper right', fontsize=8,
                                facecolor='#111', labelcolor='white')
                    _ax2.grid(True, alpha=0.1, color='#335')
                _fig2.tight_layout()
                _fig2.savefig(os.path.join(out_dir, "lidar.png"), dpi=150)
                plt.close(_fig2)

            # ── 4. Lokalizasyon karşılaştırması ────────────────────
            if self.robot.history and self.ekf.history:
                _cs3  = self.grid.cell_size
                _tr_h = self.robot.history
                _ek_h = self.ekf.history
                _dr_h = self.dr.history
                _t3   = self.timestamps
                _n3   = min(len(_tr_h), len(_ek_h), len(_t3))
                _t3   = _t3[:_n3]

                _tx  = [h[0] for h in _tr_h[:_n3]]
                _ty  = [h[1] for h in _tr_h[:_n3]]
                _ex3 = [h[0] for h in _ek_h[:_n3]]
                _ey3 = [h[1] for h in _ek_h[:_n3]]
                _dx3 = [h[0] for h in _dr_h[:_n3]] if len(_dr_h) >= _n3 else []
                _dy3 = [h[1] for h in _dr_h[:_n3]] if len(_dr_h) >= _n3 else []

                _fig3 = plt.figure(figsize=(14, 10))
                _fig3.patch.set_facecolor("#0a0f1c")
                _gs3  = _fig3.add_gridspec(2, 2, hspace=0.38, wspace=0.3)
                _axm  = _fig3.add_subplot(_gs3[0, 0])
                _axx  = _fig3.add_subplot(_gs3[0, 1])
                _axy  = _fig3.add_subplot(_gs3[1, 0])
                _axe  = _fig3.add_subplot(_gs3[1, 1])

                # 2D harita
                _axm.set_facecolor("#0a0f1c")
                for _r in range(self.grid.rows):
                    for _c in range(self.grid.cols):
                        if not self.grid.is_free(_r, _c):
                            _axm.add_patch(plt.Rectangle(
                                ((_c - 0.5) * _cs3, (_r - 0.5) * _cs3),
                                _cs3, _cs3, color='#505560', alpha=0.5))
                _axm.plot(_tx,  _ty,  '-',  c='#32C850', lw=2.0, label='Gerçek yol')
                _axm.plot(_ex3, _ey3, '--', c='#00C8DC', lw=1.5, label='EKF tahmini')
                if _dx3:
                    _axm.plot(_dx3, _dy3, ':', c='#FFD700', lw=1.2,
                              label='Dead Reckoning', alpha=0.9)
                _axm.set_xlabel("X (m)", color='#AAC')
                _axm.set_ylabel("Y (m)", color='#AAC')
                _axm.set_title("2B Konum Karşılaştırması", color='#88CCFF')
                _axm.legend(fontsize=8, facecolor='#111', labelcolor='white')
                _axm.tick_params(colors='#AAC')
                for _sp in _axm.spines.values(): _sp.set_color('#445')
                _axm.grid(True, alpha=0.1, color='#335')

                # x(t) zaman serisi
                _axx.set_facecolor("#0a0f1c")
                _axx.plot(_t3, _tx,  c='#32C850', lw=1.5, label='Gerçek x')
                _axx.plot(_t3, _ex3, '--', c='#00C8DC', lw=1.2, label='EKF x')
                if _dx3: _axx.plot(_t3, _dx3, ':', c='#FFD700', lw=1.2, label='DR x')
                _axx.set_xlabel("Zaman (s)", color='#AAC')
                _axx.set_ylabel("x (m)", color='#AAC')
                _axx.set_title("x(t) — Zaman Serisi", color='#88CCFF')
                _axx.legend(fontsize=8, facecolor='#111', labelcolor='white')
                _axx.tick_params(colors='#AAC')
                for _sp in _axx.spines.values(): _sp.set_color('#445')
                _axx.grid(True, alpha=0.1, color='#335')

                # y(t) zaman serisi
                _axy.set_facecolor("#0a0f1c")
                _axy.plot(_t3, _ty,  c='#32C850', lw=1.5, label='Gerçek y')
                _axy.plot(_t3, _ey3, '--', c='#00C8DC', lw=1.2, label='EKF y')
                if _dy3: _axy.plot(_t3, _dy3, ':', c='#FFD700', lw=1.2, label='DR y')
                _axy.set_xlabel("Zaman (s)", color='#AAC')
                _axy.set_ylabel("y (m)", color='#AAC')
                _axy.set_title("y(t) — Zaman Serisi", color='#88CCFF')
                _axy.legend(fontsize=8, facecolor='#111', labelcolor='white')
                _axy.tick_params(colors='#AAC')
                for _sp in _axy.spines.values(): _sp.set_color('#445')
                _axy.grid(True, alpha=0.1, color='#335')

                # Hata karşılaştırması
                _axe.set_facecolor("#0a0f1c")
                _errs3 = self.position_errors[:_n3]
                _axe.plot(_t3, _errs3, c='#00C8DC', lw=1.5, label='EKF hatası')
                if _dx3 and len(_dx3) >= _n3:
                    _errs_dr3 = [np.hypot(_tx[i] - _dx3[i], _ty[i] - _dy3[i])
                                 for i in range(_n3)]
                    _axe.plot(_t3, _errs_dr3, '--', c='#FFD700', lw=1.2,
                              label='DR hatası')
                if _errs3:
                    _r3 = float(np.sqrt(np.mean(np.array(_errs3) ** 2)))
                    _axe.axhline(_r3, c='#00C8DC', ls=':', alpha=0.6,
                                 label=f'EKF RMSE={_r3:.4f}m')
                _axe.set_xlabel("Zaman (s)", color='#AAC')
                _axe.set_ylabel("Hata (m)", color='#AAC')
                _axe.set_title("Lokalizasyon Hatası (EKF vs DR)", color='#88CCFF')
                _axe.legend(fontsize=8, facecolor='#111', labelcolor='white')
                _axe.tick_params(colors='#AAC')
                for _sp in _axe.spines.values(): _sp.set_color('#445')
                _axe.grid(True, alpha=0.1, color='#335')

                _fig3.suptitle(
                    f"Lokalizasyon Sonuçları — {self.robot.robot_type}"
                    f" | {self.path_stats['planner']} | {self.path_stats['criteria']}",
                    color='#88CCFF', fontsize=13)
                _fig3.savefig(os.path.join(out_dir, "lokalizasyon.png"), dpi=150)
                plt.close(_fig3)

            _saved = ["hata.png", "yol.png"]
            if self._lidar_snapshots: _saved.append("lidar.png")
            if self.robot.history:    _saved.append("lokalizasyon.png")
            print(f"  → {out_dir}/ [{', '.join(_saved)}]")

        except ImportError:
            print("matplotlib bulunamadı, grafikler kaydedilemedi.")
