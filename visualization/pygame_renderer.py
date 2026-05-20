"""
visualization/pygame_renderer.py
GCS (Yer Kontrol İstasyonu) tarzı Pygame görselleştirici.

Özellikler:
- Üst bilgi çubuğu (görev bilgisi + süre)
- Merkezlenmiş harita (koordinat etiketleri)
- Ham grid yolu (noktalı çizgi) + Yumuşatılmış yol (ayrı renk)
- Hava araçları / kara araçları için ayrı tema + panel tasarımı
- Pusula gülü (kara) / HSI göstergesi (hava)
- Hız çubuğu, EKF hatası, rota ilerleme çubuğu
- DURAKLAT / DEVAM · HIZ · TEKRAR · ANA MENÜ butonları
"""

import pygame
import numpy as np
import math
import sys
import os
from datetime import datetime
from typing import List, Tuple, Optional

# ─── Temel renkler ──────────────────────────────────────────────────────────
WHITE = (255, 255, 255)
BLACK = (0,   0,   0)

# ─── GCS paleti ─────────────────────────────────────────────────────────────
GCS_BG     = (6,   9,  14)
GCS_TOP    = (10,  15,  24)
GCS_PANEL  = (9,   13,  21)
GCS_BORDER = (0,   45,  70)
GCS_SEP    = (20,  40,  60)
GCS_TEXT   = (175, 205, 235)
GCS_DIM    = (70,  100, 130)
GCS_YELLOW = (245, 195,   0)
GCS_WARN   = (255, 150,   0)
GCS_DANGER = (220,  45,  45)
GCS_GREEN  = (55,  215,  80)
GCS_CYAN   = (0,   185, 245)

# ─── Harita renkleri ────────────────────────────────────────────────────────
MAP_BG     = (7,  11,  16)
MAP_GRID   = (17, 27,  40)
OBSTACLE_C = (52, 58,  68)
PADDING_C  = (78, 52,  18)

# ─── Yol renkleri ───────────────────────────────────────────────────────────
PATH_RAW           = (38, 72,  95)
PATH_TRVSD_AERIAL  = (0,  55,  70)
PATH_TRVSD_GROUND  = (20, 60,  30)

# ─── İz renkleri ────────────────────────────────────────────────────────────
TRUE_C  = (50,  220, 80)
EKF_C   = (0,   185, 245)
DR_C    = (255, 215, 55)
LIDAR_C = (200, 50,  50)

TRACK_COLOR = (60,  65,  78)
TRACK_BRD   = (42,  48,  62)

# ─── Robot gövde renkleri ───────────────────────────────────────────────────
BODY_COLORS = {
    "DifferentialDrive": (35,  70, 165),
    "OmniWheel":         (0,  150, 170),
    "Ackermann":         (170, 85,  10),
    "Mecanum":           (105, 28, 155),
    "Quadrotor":         (20,  155,  55),
    "VTOL":              (0,   135, 195),
    "FixedWing":         (175,  28,  28),
    "Quadruped":         (30,  140,  80),
    "Hexapod":           (160, 110,  20),
    "SnakeRobot":        (20,  145, 155),
    "Bipedal":           (145,  40, 130),
}

# ─── Canlı ayar paneli – sabit listeler ─────────────────────────────────────
_LV_ROBOTS = [
    ("differential", "Diferansiyel"),
    ("omni",         "Omniwheel"),
    ("ackermann",    "Ackermann"),
    ("mecanum",      "Mecanum"),
    ("quadruped",    "Quadruped"),
    ("hexapod",      "Hexapod"),
    ("snake",        "Yilan"),
    ("bipedal",      "Bipedal"),
    ("drone",        "Drone"),
    ("vtol",         "VTOL"),
    ("fixedwing",    "Sabit Kanat"),
]
_LV_PLANNERS = [
    ("astar",           "A*"),
    ("dijkstra",        "Dijkstra"),
    ("dstar",           "D* Lite"),
    ("rrt",             "RRT"),
    ("rrtstar",         "RRT*"),
    ("bug0",            "Bug0"),
    ("bug1",            "Bug1"),
    ("bug2",            "Bug2"),
    ("potentialfields", "Pot.Fields"),
    ("vfh",             "VFH"),
]
_LV_METRICS = [
    ("euclidean",  "Oklid"),
    ("manhattan",  "Manh."),
    ("chebyshev",  "Cheb."),
    ("octile",     "Oktil"),
    ("diagonal",   "Diag."),
    ("minkowski2", "Mink2"),
    ("minkowski3", "Mink3"),
]
_LV_CRITERIA = [
    ("shortest",  "Kisa"),
    ("safest",    "Guvenli"),
    ("fastest",   "Hizli"),
    ("smoothest", "Duzgun"),
]
_LV_ROBOT_COLORS = {
    "differential": (40,  80, 170), "omni":      (0,  155, 175),
    "ackermann":    (175, 88,  10), "mecanum":   (105,  28, 155),
    "quadruped":    (30, 140,  80), "hexapod":   (160, 110,  20),
    "snake":        (20, 145, 155), "bipedal":   (145,  40, 130),
    "drone":        (20, 160,  60), "vtol":      (0,   140, 200),
    "fixedwing":    (180, 30,  30),
}

# ─── Tema: hava / kara ──────────────────────────────────────────────────────
THEME = {
    "aerial": {
        "accent":     GCS_CYAN,
        "path":       (0,  220, 190),
        "path_dim":   PATH_TRVSD_AERIAL,
        "panel_hdr":  (8,  15,  30),
        "label":      "  HAVA ARACI",
        "compass_bg": (10, 18,  35),
        "gauge_c":    GCS_CYAN,
    },
    "ground": {
        "accent":     GCS_GREEN,
        "path":       (80, 245, 115),
        "path_dim":   PATH_TRVSD_GROUND,
        "panel_hdr":  (8,  20,  12),
        "label":      "  KARA ARACI",
        "compass_bg": (10, 25,  15),
        "gauge_c":    GCS_GREEN,
    },
}

PANEL_W = 310
TOP_H   = 42
CELL_PX = 16


class PyGameRenderer:
    """GCS tarzı gerçek zamanlı simülasyon görselleştirici."""

    AERIAL = {"Quadrotor", "VTOL", "FixedWing"}

    def __init__(self, grid_map, cell_px: int = CELL_PX, fps: int = 30,
                 screen=None, init_settings: dict = None):
        self.map   = grid_map
        self.fps   = fps
        self.paused     = False
        self.speed_mult = 1.0
        self._action    = "running"

        if screen is None:
            pygame.init()
            di = pygame.display.Info()
            W, H = di.current_w, di.current_h
            self.screen = pygame.display.set_mode((W, H), pygame.NOFRAME)
        else:
            self.screen = screen
            W, H = screen.get_size()
        pygame.display.set_caption("GCS — Otonom Navigasyon")
        self.win_w, self.win_h = W, H
        self.clock  = pygame.time.Clock()

        map_aw = W - PANEL_W
        map_ah = H - TOP_H
        opt = min(map_aw // max(grid_map.cols, 1),
                  map_ah // max(grid_map.rows, 1))
        self.cell_px = max(opt, 6)

        self.map_w  = grid_map.cols * self.cell_px
        self.map_h  = grid_map.rows * self.cell_px
        self.map_x0 = (map_aw - self.map_w) // 2
        self.map_y0 = TOP_H + (map_ah - self.map_h) // 2

        self.font_sm = pygame.font.SysFont("consolas", 12)
        self.font_md = pygame.font.SysFont("consolas", 14, bold=True)
        self.font_lg = pygame.font.SysFont("consolas", 17, bold=True)
        self.font_xl = pygame.font.SysFont("consolas", 21, bold=True)

        self._planned_path:     List[Tuple] = []
        self._smooth_path:      List[Tuple] = []
        self._smooth_remaining: List[Tuple] = []
        self._true_path:        List[Tuple] = []
        self._ekf_path:         List[Tuple] = []
        self._dr_path:          List[Tuple] = []
        self._lidar_raw         = np.empty((0, 2))
        self._lidar_points      = np.empty((0, 2))
        self._icr: Optional[tuple] = None
        self._hud_info: dict    = {}

        self._robot_type  = "DifferentialDrive"
        self._is_aerial   = False
        self._theme       = THEME["ground"]
        self._wp_total    = 1
        self._wp_current  = 0
        self._course_theta: Optional[float] = None

        self._map_surface = self._build_map_surface()
        self._build_buttons()
        _s = init_settings or {}
        self._live = {
            "robot":    _s.get("robot",    "differential"),
            "planner":  _s.get("planner",  "astar"),
            "metric":   _s.get("metric",   "euclidean"),
            "criteria": _s.get("criteria", "shortest"),
        }
        self._build_live_btns()

    # ─── Dış API ────────────────────────────────────────────────────

    def set_robot_type(self, robot_type: str):
        self._robot_type = robot_type
        self._is_aerial  = robot_type in self.AERIAL
        self._theme      = THEME["aerial" if self._is_aerial else "ground"]

    def set_planned_path(self, path: List[Tuple[int, int]]):
        self._planned_path = [self._rc_to_px(r, c) for r, c in path]
        self._wp_total = max(len(path), 1)

    def set_smooth_path(self, path_world: List[Tuple[float, float]]):
        self._smooth_path      = [self._world_to_px(x, y) for x, y in path_world]
        self._smooth_remaining = list(self._smooth_path)
        self._wp_total         = max(len(path_world), 1)

    def update(self, robot, lidar_raw: np.ndarray,
               lidar_filtered: np.ndarray,
               ekf_pose: Tuple = None,
               dr_pose: Tuple = None,
               hud_extra: dict = None,
               wp_idx: int = 0,
               icr: tuple = None) -> str:
        """Returns: 'running'|'paused'|'replay'|'menu'|'quit'"""
        self._icr         = icr
        self._wp_current  = wp_idx

        if 0 <= wp_idx < len(self._smooth_path):
            self._smooth_remaining = self._smooth_path[wp_idx:]
        else:
            self._smooth_remaining = []

        if robot.history:
            x, y, _ = robot.history[-1]
            self._true_path.append(self._world_to_px(x, y))
        if ekf_pose:
            self._ekf_path.append(self._world_to_px(ekf_pose[0], ekf_pose[1]))
        if dr_pose:
            self._dr_path.append(self._world_to_px(dr_pose[0], dr_pose[1]))

        self._lidar_raw    = lidar_raw
        self._lidar_points = lidar_filtered
        self._hud_info     = dict(hud_extra or {})
        self._hud_info.update({
            "Robot": robot.robot_type,
            "Konum": f"({robot.state.x:.2f},{robot.state.y:.2f})",
            "Ac":    f"{math.degrees(robot.state.theta):.1f}",
            "Hz":    f"{math.hypot(robot.state.vx, robot.state.vy):.2f} m/s",
        })

        self._draw(robot)
        self._handle_events()
        return self._action

    def handle_paused(self) -> str:
        txt = self.font_xl.render("DURAKLATILDI  --  SPACE ile devam", True, GCS_YELLOW)
        rx = self.win_w // 2 - txt.get_width() // 2
        ry = self.map_y0 + self.map_h // 2 - txt.get_height() // 2
        pygame.draw.rect(self.screen, (10, 15, 28),
                         pygame.Rect(rx - 16, ry - 10, txt.get_width() + 32, txt.get_height() + 20),
                         border_radius=8)
        pygame.draw.rect(self.screen, GCS_YELLOW,
                         pygame.Rect(rx - 16, ry - 10, txt.get_width() + 32, txt.get_height() + 20),
                         1, border_radius=8)
        self.screen.blit(txt, (rx, ry))
        self._draw_buttons(self.win_w - PANEL_W)
        pygame.display.flip()
        self.clock.tick(30)
        self._handle_events()
        return self._action

    def show_arrived(self, elapsed_time: float, position_errors: list) -> str:
        errs = np.array(position_errors) if position_errors else np.array([0.0])
        rmse = float(np.sqrt(np.mean(errs ** 2)))
        mae  = float(np.mean(np.abs(errs)))

        bw   = PANEL_W - 20
        bh   = 40
        bx   = self.win_w - PANEL_W + 10
        btn_r = pygame.Rect(bx, self.win_h - 2 * (bh + 8) - 10, bw, bh)
        btn_m = pygame.Rect(bx, self.win_h - (bh + 10), bw, bh)

        card_w = min(480, self.map_w - 40)
        card_h = 160
        card_x = self.map_x0 + (self.map_w - card_w) // 2
        card_y = self.map_y0 + (self.map_h - card_h) // 2

        while True:
            ov = pygame.Surface((self.win_w, self.win_h), pygame.SRCALPHA)
            ov.fill((0, 0, 0, 150))
            self.screen.blit(ov, (0, 0))

            card = pygame.Rect(card_x, card_y, card_w, card_h)
            pygame.draw.rect(self.screen, (8, 18, 12), card, border_radius=12)
            pygame.draw.rect(self.screen, GCS_GREEN, card, 2, border_radius=12)

            t1 = self.font_xl.render("HEDEFE ULASILDI", True, GCS_GREEN)
            t2 = self.font_md.render(
                f"Sure: {elapsed_time:.1f}s   |   RMSE: {rmse:.3f} m   |   MAE: {mae:.3f} m",
                True, GCS_TEXT)
            t3 = self.font_sm.render("R = Tekrar  |  M = Ana Menu", True, GCS_DIM)
            self.screen.blit(t1, (card_x + (card_w - t1.get_width()) // 2, card_y + 18))
            self.screen.blit(t2, (card_x + (card_w - t2.get_width()) // 2, card_y + 68))
            self.screen.blit(t3, (card_x + (card_w - t3.get_width()) // 2, card_y + 106))

            mx, my = pygame.mouse.get_pos()
            for btn, label, c_n, c_h, brd in [
                (btn_r, "TEKRAR OYNAT", (20, 35, 75), (40, 65, 135), (80, 130, 200)),
                (btn_m, "ANA MENU",    (28, 14, 48), (52, 28,  90), (140, 90, 200)),
            ]:
                hover = btn.collidepoint(mx, my)
                pygame.draw.rect(self.screen, c_h if hover else c_n, btn, border_radius=6)
                pygame.draw.rect(self.screen, brd, btn, 1, border_radius=6)
                lt = self.font_md.render(label, True, WHITE)
                self.screen.blit(lt, (btn.x + (btn.w - lt.get_width()) // 2,
                                      btn.y + (btn.h - lt.get_height()) // 2))

            pygame.display.flip()
            self.clock.tick(30)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return "quit"
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        return "replay"
                    if event.key in (pygame.K_m, pygame.K_ESCAPE):
                        return "menu"
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if btn_r.collidepoint(event.pos):
                        return "replay"
                    if btn_m.collidepoint(event.pos):
                        return "menu"

    def draw_static(self, start_rc, goal_rc, path=None):
        if path:
            self.set_planned_path(path)
        self.screen.fill(GCS_BG)
        self._draw_top_bar()
        pygame.draw.rect(self.screen, MAP_BG,
                         pygame.Rect(0, TOP_H, self.win_w - PANEL_W, self.win_h - TOP_H))
        pygame.draw.rect(self.screen, GCS_BORDER,
                         pygame.Rect(self.map_x0 - 1, self.map_y0 - 1,
                                     self.map_w + 2, self.map_h + 2), 1)
        self.screen.blit(self._map_surface, (self.map_x0, self.map_y0))
        pygame.display.flip()

    def close(self):
        pass  # pencere ana döngü tarafından yönetilir

    def _take_screenshot(self):
        """F12 ile mevcut ekranı PNG olarak kaydet."""
        out = os.path.join("outputs", "screenshots")
        os.makedirs(out, exist_ok=True)
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]
        path = os.path.join(out, f"ekran_{ts}.png")
        pygame.image.save(self.screen, path)
        print(f"  📸 Ekran görüntüsü kaydedildi → {path}")

    def tick(self):
        self.clock.tick(int(self.fps * self.speed_mult))

    # ─── Buton oluşturma ────────────────────────────────────────────

    def _build_buttons(self):
        px = self.win_w - PANEL_W + 8
        bw = PANEL_W - 16
        hb = PANEL_W // 2 - 10
        bh = 36
        y  = self.win_h - 5 * bh - 4 * 7 - 10
        self._btns = {
            "pause":  pygame.Rect(px,      y,            bw, bh),
            "faster": pygame.Rect(px,      y + bh + 7,   hb, bh),
            "slower": pygame.Rect(px+hb+4, y + bh + 7,   hb, bh),
            "replay": pygame.Rect(px,      y + 2*(bh+7), bw, bh),
            "menu":   pygame.Rect(px,      y + 3*(bh+7), bw, bh),
        }

    def _build_live_btns(self):
        """Sol kenar canlı ayar paneli için buton rect'lerini oluşturur."""
        bh = 18; gap = 2; col_gap = 3
        lm_w = max(self.map_x0 - 8, 40)
        bw2  = max((lm_w - col_gap) // 2, 28)
        lx0  = 4
        lx1  = lx0 + bw2 + col_gap

        # Zoom panel geometrisi (aynı formül)
        zm  = min(self.map_w // 3, 260, lm_w)
        zy  = self.map_y0 + (self.map_h - zm) // 2

        # ── Zoom ÜSTÜ: Robot tipi ──────────────────────────────────
        y = self.map_y0 + 4
        self._lv_robot_hdr_y = y
        y += 14
        self._lv_robot_btns = []
        for i, (key, lbl) in enumerate(_LV_ROBOTS):
            c, row = i % 2, i // 2
            bx = lx0 if c == 0 else lx1
            self._lv_robot_btns.append(
                (key, lbl, pygame.Rect(bx, y + row * (bh + gap), bw2, bh)))

        # ── Zoom ALTI: Planlayıcı ──────────────────────────────────
        y2 = zy + zm + 10
        self._lv_plan_hdr_y = y2
        y2 += 14
        self._lv_plan_btns = []
        for i, (key, lbl) in enumerate(_LV_PLANNERS):
            c, row = i % 2, i // 2
            bx = lx0 if c == 0 else lx1
            self._lv_plan_btns.append(
                (key, lbl, pygame.Rect(bx, y2 + row * (bh + gap), bw2, bh)))
        n_plan_rows = (len(_LV_PLANNERS) + 1) // 2
        y2 += n_plan_rows * (bh + gap) + 6

        # ── Metrik ────────────────────────────────────────────────
        self._lv_met_hdr_y = y2
        y2 += 14
        self._lv_met_btns = []
        for i, (key, lbl) in enumerate(_LV_METRICS):
            c, row = i % 2, i // 2
            bx = lx0 if c == 0 else lx1
            self._lv_met_btns.append(
                (key, lbl, pygame.Rect(bx, y2 + row * (bh + gap), bw2, bh)))
        n_met_rows = (len(_LV_METRICS) + 1) // 2
        y2 += n_met_rows * (bh + gap) + 6

        # ── Kriter ────────────────────────────────────────────────
        self._lv_crit_hdr_y = y2
        y2 += 14
        self._lv_crit_btns = []
        for i, (key, lbl) in enumerate(_LV_CRITERIA):
            c, row = i % 2, i // 2
            bx = lx0 if c == 0 else lx1
            self._lv_crit_btns.append(
                (key, lbl, pygame.Rect(bx, y2 + row * (bh + gap), bw2, bh)))

    def _draw_live_panel(self):
        """Sol kenar boşluğundaki canlı ayarlar panelini çizer."""
        lm_w = self.map_x0 - 8
        if lm_w < 40:
            return

        mp  = pygame.mouse.get_pos()
        acc = self._theme["accent"]

        def _hdr(y, text, color):
            pygame.draw.line(self.screen,
                             tuple(max(c // 4, 10) for c in color),
                             (4, y + 6), (lm_w, y + 6), 1)
            t  = self.font_sm.render(text, True, color)
            bg = pygame.Rect(4, y, t.get_width() + 6, t.get_height() + 2)
            pygame.draw.rect(self.screen, tuple(c // 6 for c in color), bg,
                             border_radius=3)
            self.screen.blit(t, (7, y + 1))

        def _btns(items, sel_key, sel_col):
            for key, lbl, r in items:
                if r.y + r.h > self.win_h - 2:
                    continue
                is_s = (key == sel_key)
                is_h = r.collidepoint(mp)
                bg  = (tuple(min(c // 4 + 18, 68) for c in sel_col)
                       if is_s else ((26, 42, 68) if is_h else (12, 20, 34)))
                brd = (sel_col if is_s
                       else ((35, 58, 95) if is_h else (22, 38, 62)))
                pygame.draw.rect(self.screen, bg,  r, border_radius=4)
                pygame.draw.rect(self.screen, brd, r,
                                 2 if is_s else 1, border_radius=4)
                tc = (215, 235, 255) if is_s else (72, 108, 150)
                t  = self.font_sm.render(lbl, True, tc)
                self.screen.blit(t, t.get_rect(midleft=(r.x + 4, r.centery)))

        # Robot tipi (zoom üstü)
        _hdr(self._lv_robot_hdr_y, "ARAC TIPI", acc)
        r_col = _LV_ROBOT_COLORS.get(self._live.get("robot", ""), acc)
        _btns(self._lv_robot_btns, self._live.get("robot", ""), r_col)

        # Planlayıcı (zoom altı)
        _local = {"bug0", "bug1", "bug2", "potentialfields", "vfh"}
        p_col  = GCS_CYAN if self._live.get("planner", "") in _local else acc
        _hdr(self._lv_plan_hdr_y, "PLANLAYICI", p_col)
        _btns(self._lv_plan_btns, self._live.get("planner", ""), p_col)

        # Metrik
        _hdr(self._lv_met_hdr_y, "METRIK", GCS_GREEN)
        _btns(self._lv_met_btns, self._live.get("metric", ""), GCS_GREEN)

        # Kriter
        _hdr(self._lv_crit_hdr_y, "KRITER", GCS_WARN)
        _btns(self._lv_crit_btns, self._live.get("criteria", ""), GCS_WARN)

        # İpucu
        hint = self.font_sm.render("R = YENIDEN UYGULA", True, GCS_DIM)
        self.screen.blit(hint, (4, self.win_h - hint.get_height() - 4))

    # ─── Olay işleme ────────────────────────────────────────────────

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._action = "quit"; return
            if event.type == pygame.KEYDOWN:
                k = event.key
                if k == pygame.K_ESCAPE:
                    self._action = "quit"
                elif k == pygame.K_SPACE:
                    self.paused = not self.paused
                    self._action = "paused" if self.paused else "running"
                elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    self.speed_mult = min(self.speed_mult * 1.5, 10.0)
                elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    self.speed_mult = max(self.speed_mult / 1.5, 0.2)
                elif k == pygame.K_r:
                    self._action = "replay"
                elif k == pygame.K_m:
                    self._action = "menu"
                elif k == pygame.K_F12:
                    self._take_screenshot()
            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = event.pos
                if self._btns["pause"].collidepoint(pos):
                    self.paused = not self.paused
                    self._action = "paused" if self.paused else "running"
                elif self._btns["faster"].collidepoint(pos):
                    self.speed_mult = min(self.speed_mult * 1.5, 10.0)
                elif self._btns["slower"].collidepoint(pos):
                    self.speed_mult = max(self.speed_mult / 1.5, 0.2)
                elif self._btns["replay"].collidepoint(pos):
                    self._action = "replay"
                elif self._btns["menu"].collidepoint(pos):
                    self._action = "menu"
                # ─ Canlı ayarlar ─
                for key, _, r in self._lv_robot_btns:
                    if r.collidepoint(pos):
                        self._live["robot"] = key
                for key, _, r in self._lv_plan_btns:
                    if r.collidepoint(pos):
                        self._live["planner"] = key
                for key, _, r in self._lv_met_btns:
                    if r.collidepoint(pos):
                        self._live["metric"] = key
                for key, _, r in self._lv_crit_btns:
                    if r.collidepoint(pos):
                        self._live["criteria"] = key


    # ─── Ana çizim ───────────────────────────────────────────────────

    def _draw(self, robot):
        self.screen.fill(GCS_BG)
        self._draw_top_bar()
        pygame.draw.rect(self.screen, MAP_BG,
                         pygame.Rect(0, TOP_H, self.win_w - PANEL_W, self.win_h - TOP_H))
        pygame.draw.rect(self.screen, GCS_BORDER,
                         pygame.Rect(self.map_x0 - 1, self.map_y0 - 1,
                                     self.map_w + 2, self.map_h + 2), 1)
        self.screen.blit(self._map_surface, (self.map_x0, self.map_y0))
        self._draw_map_coords()
        self._draw_paths()
        self._draw_lidar(robot.state.x, robot.state.y)
        self._draw_robot(robot)
        self._draw_zoom_panel(robot)
        self._draw_live_panel()
        if self._icr is not None:
            self._draw_icr(robot)
        self._draw_panel(robot)
        pygame.display.flip()
        self.clock.tick(int(self.fps * self.speed_mult))

    # ─── Üst çubuk ───────────────────────────────────────────────────

    def _draw_top_bar(self):
        pygame.draw.rect(self.screen, GCS_TOP, pygame.Rect(0, 0, self.win_w, TOP_H))
        pygame.draw.line(self.screen, GCS_BORDER, (0, TOP_H - 1), (self.win_w, TOP_H - 1))

        accent = self._theme["accent"]
        t = self.font_lg.render("GCS v2.0", True, accent)
        self.screen.blit(t, (14, (TOP_H - t.get_height()) // 2))

        planner = self._hud_info.get("Planlayici", "---")
        metric  = self._hud_info.get("Metrik",     "---")
        kriter  = self._hud_info.get("Kriter",     "---")
        info = f"Planlayici: {planner}  |  Metrik: {metric}  |  Kriter: {kriter}"
        ti = self.font_sm.render(info, True, GCS_DIM)
        self.screen.blit(ti, (self.win_w // 2 - ti.get_width() // 2,
                               (TOP_H - ti.get_height()) // 2))

        pause_tag = "  [DURAKLATILDI]" if self.paused else ""
        time_str  = self._hud_info.get("Sure", "0.0 s")
        ts = self.font_md.render(f"Sure: {time_str}{pause_tag}", True, GCS_YELLOW)
        self.screen.blit(ts, (self.win_w - PANEL_W - ts.get_width() - 16,
                               (TOP_H - ts.get_height()) // 2))

    # ─── Harita koordinat etiketleri ─────────────────────────────────

    def _draw_map_coords(self):
        step = 5
        for c in range(0, self.map.cols + 1, step):
            x = self.map_x0 + c * self.cell_px
            t = self.font_sm.render(str(c), True, GCS_DIM)
            self.screen.blit(t, (x - t.get_width() // 2, self.map_y0 - 14))
        for r in range(0, self.map.rows + 1, step):
            y = self.map_y0 + r * self.cell_px
            t = self.font_sm.render(str(r), True, GCS_DIM)
            self.screen.blit(t, (self.map_x0 - t.get_width() - 4,
                                  y - t.get_height() // 2))

    # ─── Yollar ──────────────────────────────────────────────────────

    def _draw_paths(self):
        # 1. Ham grid yolu — ince + noktalar
        if len(self._planned_path) > 1:
            pygame.draw.lines(self.screen, (30, 55, 75), False,
                               self._planned_path, 1)
            for i, pt in enumerate(self._planned_path):
                if i % 2 == 0:
                    pygame.draw.circle(self.screen, PATH_RAW, pt, 2)

        # 2. Yumuşak yol — geçilen (soluk)
        trvsd_end = len(self._smooth_path) - len(self._smooth_remaining)
        if trvsd_end > 1:
            pygame.draw.lines(self.screen, self._theme["path_dim"],
                               False, self._smooth_path[:trvsd_end + 1], 2)

        # 3. Yumuşak yol — kalan (parlak)
        if len(self._smooth_remaining) > 1:
            pygame.draw.lines(self.screen, self._theme["path"],
                               False, self._smooth_remaining, 3)

        # 4. Dead reckoning
        if len(self._dr_path) > 1:
            pygame.draw.lines(self.screen, DR_C,   False, self._dr_path, 1)
        # 5. EKF
        if len(self._ekf_path) > 1:
            pygame.draw.lines(self.screen, EKF_C,  False, self._ekf_path, 2)
        # 6. Gerçek iz
        if len(self._true_path) > 1:
            pygame.draw.lines(self.screen, TRUE_C, False, self._true_path, 2)

    # ─── LiDAR ───────────────────────────────────────────────────────

    def _draw_lidar(self, rx: float, ry: float):
        rx_px, ry_px = self._world_to_px(rx, ry)
        for pt in self._lidar_raw:
            px2, py2 = self._world_to_px(pt[0], pt[1])
            pygame.draw.line(self.screen, (70, 20, 20), (rx_px, ry_px), (px2, py2), 1)
        for pt in self._lidar_points:
            px2, py2 = self._world_to_px(pt[0], pt[1])
            pygame.draw.circle(self.screen, LIDAR_C, (px2, py2), 2)

    # ─── Robot çizimi ────────────────────────────────────────────────

    def _draw_robot(self, robot):
        x_px, y_px = self._world_to_px(robot.state.x, robot.state.y)
        rtype  = robot.robot_type
        theta  = robot.state.theta
        scale  = self.cell_px / self.map.cell_size
        r_px   = max(int(robot.radius * scale), 5)
        cos_t  = math.cos(theta)
        sin_t  = math.sin(theta)
        body_c = BODY_COLORS.get(rtype, (60, 100, 200))

        def rot(cx, cy):
            return (int(cx * cos_t - cy * sin_t + x_px),
                    int(cx * sin_t + cy * cos_t + y_px))

        if rtype in ("DifferentialDrive", "Mecanum"):
            bw = r_px * 2.2; bh = r_px * 1.6
            tw = r_px * 2.4; th = r_px * 0.55
            for side in (-1, 1):
                oy = side * (bh / 2 + th / 2 + 1)
                tp = [rot(-tw/2, oy-th/2), rot(tw/2, oy-th/2),
                      rot( tw/2, oy+th/2), rot(-tw/2, oy+th/2)]
                pygame.draw.polygon(self.screen, TRACK_COLOR, tp)
                pygame.draw.polygon(self.screen, TRACK_BRD, tp, 1)
                for seg in range(-3, 4):
                    sx = seg * tw / 7
                    sp = [rot(sx-1, oy-th/2+1), rot(sx+1, oy-th/2+1),
                          rot(sx+1, oy+th/2-1), rot(sx-1, oy+th/2-1)]
                    pygame.draw.polygon(self.screen, TRACK_BRD, sp, 1)
            bp = [rot(-bw/2,-bh/2), rot(bw/2,-bh/2),
                  rot( bw/2, bh/2), rot(-bw/2, bh/2)]
            pygame.draw.polygon(self.screen, body_c, bp)
            pygame.draw.polygon(self.screen, WHITE, bp, 2)
            tr = int(r_px * 0.55)
            pygame.draw.circle(self.screen, tuple(min(c+40,255) for c in body_c),
                                (x_px, y_px), tr)
            pygame.draw.circle(self.screen, WHITE, (x_px, y_px), tr, 1)
            bx_ = int(x_px + r_px * 1.6 * cos_t)
            by_ = int(y_px + r_px * 1.6 * sin_t)
            pygame.draw.line(self.screen, WHITE, (x_px, y_px), (bx_, by_), 3)

        elif rtype == "OmniWheel":
            pygame.draw.circle(self.screen, body_c, (x_px, y_px), r_px)
            pygame.draw.circle(self.screen, WHITE, (x_px, y_px), r_px, 2)
            for i in range(3):
                ang = theta + i * (2 * math.pi / 3)
                w1 = (int(x_px + r_px * math.cos(ang)),
                      int(y_px + r_px * math.sin(ang)))
                w2 = (int(x_px + (r_px+5) * math.cos(ang)),
                      int(y_px + (r_px+5) * math.sin(ang)))
                pygame.draw.line(self.screen, GCS_YELLOW, w1, w2, 4)
            nx = int(x_px + r_px * 0.9 * cos_t)
            ny = int(y_px + r_px * 0.9 * sin_t)
            pygame.draw.circle(self.screen, WHITE, (nx, ny), 4)
            pygame.draw.line(self.screen, WHITE, (x_px, y_px), (nx, ny), 2)

        elif rtype == "Ackermann":
            bw = r_px * 2.0; bh = r_px * 1.3
            bp = [rot(-bw/2,-bh/2), rot(bw/2,-bh/2),
                  rot( bw/2, bh/2), rot(-bw/2, bh/2)]
            pygame.draw.polygon(self.screen, body_c, bp)
            pygame.draw.polygon(self.screen, WHITE, bp, 2)
            for axle_x in (-bw*0.35, bw*0.35):
                pygame.draw.line(self.screen, (140,140,140),
                                 rot(axle_x, -bh/2-4), rot(axle_x, bh/2+4), 3)
            ex_ = int(x_px + r_px * 1.5 * cos_t)
            ey_ = int(y_px + r_px * 1.5 * sin_t)
            pygame.draw.line(self.screen, WHITE, (x_px, y_px), (ex_, ey_), 3)
            pygame.draw.circle(self.screen, WHITE, (ex_, ey_), 3)

        elif rtype == "Quadrotor":
            arm = r_px * 2.5; rr = max(int(r_px*0.60), 4); br = max(int(r_px*0.55), 5)
            for i in range(4):
                aa  = theta + math.pi/4 + i * math.pi/2
                tip = (int(x_px + arm * math.cos(aa)),
                       int(y_px + arm * math.sin(aa)))
                bx2 = int(x_px + br * math.cos(aa))
                by2 = int(y_px + br * math.sin(aa))
                pygame.draw.line(self.screen, (140,140,140), (bx2,by2), tip, 2)
                pygame.draw.circle(self.screen, (28, 28, 38), tip, rr)
                pygame.draw.circle(self.screen, body_c, tip, rr, 1)
                for b2 in (theta*4 + i*math.pi/2, theta*4 + i*math.pi/2 + math.pi/2):
                    pygame.draw.line(self.screen, body_c,
                        (int(tip[0]+rr*0.8*math.cos(b2)), int(tip[1]+rr*0.8*math.sin(b2))),
                        (int(tip[0]-rr*0.8*math.cos(b2)), int(tip[1]-rr*0.8*math.sin(b2))), 2)
            pygame.draw.circle(self.screen, body_c, (x_px, y_px), br)
            pygame.draw.circle(self.screen, WHITE, (x_px, y_px), br, 2)
            led = rot(br, 0)
            pygame.draw.circle(self.screen, (220, 50, 50), led, 3)

        elif rtype == "VTOL":
            fl = r_px*2.5; fh = r_px*0.50; wl = r_px*2.0
            wp = [rot(-fl*0.1,-wl), rot(fl*0.2,-wl),
                  rot( fl*0.2, wl), rot(-fl*0.1, wl)]
            pygame.draw.polygon(self.screen,
                                tuple(max(c-20,0) for c in body_c), wp)
            pygame.draw.polygon(self.screen, WHITE, wp, 1)
            bp = [rot(-fl/2,-fh/2), rot(fl/2,-fh/2),
                  rot( fl/2, fh/2), rot(-fl/2, fh/2)]
            pygame.draw.polygon(self.screen, body_c, bp)
            pygame.draw.polygon(self.screen, WHITE, bp, 2)
            rr = max(int(r_px*0.35), 4)
            for side in (-1, 1):
                rx2 = rot(0, int(side*wl*0.82))
                pygame.draw.circle(self.screen, (25,25,40), rx2, rr)
                pygame.draw.circle(self.screen, (0,230,160), rx2, rr, 1)
            nose = rot(int(fl*0.55), 0)
            pygame.draw.circle(self.screen, GCS_YELLOW, nose, 4)

        elif rtype == "FixedWing":
            fl=r_px*2.8; fh=r_px*0.45; wl=r_px*2.2; sl=r_px*0.8
            for side in (-1,1):
                wp2 = [rot(-fl*0.05,int(side*fh*0.5)), rot( fl*0.20,int(side*fh*0.5)),
                       rot(-fl*0.18,int(side*(fh*0.5+wl))), rot(-fl*0.30,int(side*(fh*0.5+wl)))]
                pygame.draw.polygon(self.screen, body_c, wp2)
                pygame.draw.polygon(self.screen, WHITE, wp2, 1)
                tp2 = [rot(-fl*0.38,int(side*fh*0.4)), rot(-fl*0.30,int(side*fh*0.4)),
                       rot(-fl*0.38,int(side*(fh*0.4+sl)))]
                pygame.draw.polygon(self.screen,
                                    tuple(max(c-20,0) for c in body_c), tp2)
                pygame.draw.polygon(self.screen, WHITE, tp2, 1)
            bp = [rot(-fl/2,-fh/2), rot(fl/2,-fh/2),
                  rot( fl/2, fh/2), rot(-fl/2, fh/2)]
            pygame.draw.polygon(self.screen,
                                tuple(min(c+20,255) for c in body_c), bp)
            pygame.draw.polygon(self.screen, WHITE, bp, 2)
            vt = [rot(-fl*0.45,0), rot(-fl*0.25,0), rot(-fl*0.42,-int(sl*0.7))]
            pygame.draw.polygon(self.screen,
                                tuple(min(c+30,255) for c in body_c), vt)
            pygame.draw.polygon(self.screen, WHITE, vt, 1)
            nose = rot(int(fl*0.52), 0)
            pygame.draw.circle(self.screen, GCS_YELLOW, nose, 4)
        else:
            pygame.draw.circle(self.screen, body_c, (x_px, y_px), r_px)
            pygame.draw.circle(self.screen, WHITE, (x_px, y_px), r_px, 2)
            ex_ = int(x_px + r_px*1.3*cos_t); ey_ = int(y_px + r_px*1.3*sin_t)
            pygame.draw.line(self.screen, WHITE, (x_px,y_px), (ex_,ey_), 3)

    # ─── ICR ─────────────────────────────────────────────────────────

    def _draw_icr(self, robot):
        icr_x, icr_y, R = self._icr
        scale = self.cell_px / self.map.cell_size
        rx_px, ry_px = self._world_to_px(robot.state.x, robot.state.y)
        ix_px, iy_px = self._world_to_px(icr_x, icr_y)
        pygame.draw.line(self.screen, GCS_YELLOW, (rx_px,ry_px), (ix_px,iy_px), 1)
        if 0 <= ix_px < self.win_w and 0 <= iy_px < self.win_h:
            pygame.draw.circle(self.screen, GCS_YELLOW, (ix_px,iy_px), 6)
            lbl = self.font_sm.render("ICR", True, GCS_YELLOW)
            self.screen.blit(lbl, (ix_px+10, iy_px-8))
        r_px_cir = int(abs(R) * scale)
        if 8 < r_px_cir < 900:
            rect = pygame.Rect(ix_px-r_px_cir, iy_px-r_px_cir,
                               r_px_cir*2, r_px_cir*2)
            atg = math.atan2(-(ry_px-iy_px), rx_px-ix_px)
            try:
                pygame.draw.arc(self.screen, GCS_YELLOW, rect,
                                atg-0.7, atg+0.7, 1)
            except Exception:
                pass
        lt = self.font_sm.render(f"R={abs(R):.1f}m", True, GCS_YELLOW)
        self.screen.blit(lt, (ix_px+10, iy_px+8))

    # ─── Sağ panel ───────────────────────────────────────────────────

    def _draw_panel(self, robot):
        px  = self.win_w - PANEL_W
        acc = self._theme["accent"]
        pygame.draw.rect(self.screen, GCS_PANEL,
                         pygame.Rect(px, TOP_H, PANEL_W, self.win_h - TOP_H))
        pygame.draw.line(self.screen, GCS_BORDER, (px, TOP_H), (px, self.win_h), 2)

        y = TOP_H + 4

        # ── Araç başlığı ─────────────────────────────────────────────
        pygame.draw.rect(self.screen, self._theme["panel_hdr"],
                         pygame.Rect(px, y, PANEL_W, 44))
        pygame.draw.line(self.screen, acc, (px, y+44), (px+PANEL_W, y+44), 1)
        lbl = self.font_lg.render(self._theme["label"], True, acc)
        typ = self.font_sm.render(self._robot_type, True, (170,175,185))
        self.screen.blit(lbl, (px + (PANEL_W - lbl.get_width()) // 2, y + 4))
        self.screen.blit(typ, (px + (PANEL_W - typ.get_width()) // 2, y + 28))
        y += 52

        # ── Telemetri ────────────────────────────────────────────────
        y = self._section_title(px, y, "TELEMETRI")
        speed = math.hypot(robot.state.vx, robot.state.vy)
        y = self._kv(px, y, "Konum",
                     f"({robot.state.x:.2f}, {robot.state.y:.2f}) m")
        y = self._kv(px, y, "Yon",
                     f"{math.degrees(robot.state.theta):.1f} deg")
        y = self._speed_gauge(px, y, speed, robot.max_linear_speed)
        y += 4

        # Pusula / HSI
        cx_comp = px + PANEL_W // 2
        if self._is_aerial:
            y = self._draw_hsi(cx_comp, y + 42, robot.state.theta, y)
        else:
            y = self._draw_compass(cx_comp, y + 42, robot.state.theta, y)
        y += 6

        # ICR (kara araçları)
        if not self._is_aerial and self._icr is not None:
            icr_x, icr_y, R = self._icr
            y = self._kv(px, y, "ICR R", f"{abs(R):.2f} m")

        # EKF hatası
        ekf_err = self._hud_info.get("EKF err", "---")
        y = self._kv(px, y, "EKF", ekf_err, EKF_C)
        y += 4

        # ── Rota bilgisi ─────────────────────────────────────────────
        pygame.draw.line(self.screen, GCS_SEP,
                         (px+6, y), (px+PANEL_W-6, y)); y += 7
        y = self._section_title(px, y, "ROTA")
        wp_tot = max(self._wp_total, 1)
        wp_cur = min(self._wp_current, wp_tot)
        pct    = wp_cur / wp_tot
        y = self._kv(px, y, "Waypoint", f"{wp_cur} / {wp_tot}")
        y = self._progress_bar(px, y, pct, acc)
        y += 4

        # ── Açıklama ─────────────────────────────────────────────────
        pygame.draw.line(self.screen, GCS_SEP,
                         (px+6, y), (px+PANEL_W-6, y)); y += 7
        y = self._section_title(px, y, "ACIKLAMA")
        for color, text in [
            (PATH_RAW,             ".. Ham grid yolu"),
            (self._theme["path"],  "--- Kalan duzeltilmis"),
            (TRUE_C,               "--- Gercek iz"),
            (EKF_C,                "--- EKF tahmini"),
            (DR_C,                 "--- Dead reckoning"),
            (LIDAR_C,              ".. LiDAR"),
        ]:
            t = self.font_sm.render(text, True, color)
            self.screen.blit(t, (px + 10, y)); y += 16
        y += 2

        # ── Hız çarpanı ──────────────────────────────────────────────
        hz_t = self.font_sm.render(
            f"Sim hizi: x{self.speed_mult:.1f}  (+/-)", True, GCS_DIM)
        self.screen.blit(hz_t, (px + (PANEL_W - hz_t.get_width()) // 2, y))

        # ── Butonlar ─────────────────────────────────────────────────
        self._draw_buttons(px)

    # ─── Panel yardımcılar ────────────────────────────────────────────

    def _section_title(self, px, y, text):
        t = self.font_md.render(text, True, self._theme["accent"])
        self.screen.blit(t, (px + 8, y))
        return y + 20

    def _kv(self, px, y, key, val, val_c=None):
        kt = self.font_sm.render(f"{key}:", True, GCS_DIM)
        vt = self.font_sm.render(val, True, val_c if val_c else GCS_TEXT)
        self.screen.blit(kt, (px + 10, y))
        self.screen.blit(vt, (px + PANEL_W - vt.get_width() - 10, y))
        return y + 18

    def _speed_gauge(self, px, y, speed, max_speed):
        bar_w = PANEL_W - 18; bar_h = 13; bx = px + 9
        pct   = min(speed / max(max_speed, 0.01), 1.0)
        pygame.draw.rect(self.screen, (18,28,40),
                         pygame.Rect(bx, y, bar_w, bar_h), border_radius=3)
        if pct > 0:
            fc = (self._theme["gauge_c"] if pct < 0.7 else
                  GCS_WARN if pct < 0.9 else GCS_DANGER)
            pygame.draw.rect(self.screen, fc,
                             pygame.Rect(bx, y, int(bar_w*pct), bar_h),
                             border_radius=3)
        pygame.draw.rect(self.screen, GCS_SEP,
                         pygame.Rect(bx, y, bar_w, bar_h), 1, border_radius=3)
        vt = self.font_sm.render(f"Hiz: {speed:.2f} m/s", True, GCS_TEXT)
        self.screen.blit(vt, (bx, y + bar_h + 2))
        return y + bar_h + 18

    def _progress_bar(self, px, y, pct, accent):
        bar_w = PANEL_W - 18; bar_h = 11; bx = px + 9
        pygame.draw.rect(self.screen, (18,28,40),
                         pygame.Rect(bx, y, bar_w, bar_h), border_radius=3)
        if pct > 0:
            pygame.draw.rect(self.screen, accent,
                             pygame.Rect(bx, y, int(bar_w*pct), bar_h),
                             border_radius=3)
        pygame.draw.rect(self.screen, GCS_SEP,
                         pygame.Rect(bx, y, bar_w, bar_h), 1, border_radius=3)
        t = self.font_sm.render(f"%{int(pct*100)} tamamlandi", True, GCS_DIM)
        self.screen.blit(t, (bx, y + bar_h + 2))
        return y + bar_h + 18

    def _draw_compass(self, cx, cy_center, theta, y_base):
        """Kara araçları için pusula gülü."""
        R  = 34; cy = cy_center
        acc = self._theme["accent"]
        pygame.draw.circle(self.screen, self._theme["compass_bg"], (cx, cy), R)
        for i in range(8):
            ang = math.radians(i * 45)
            r2  = R - (9 if i % 2 == 0 else 5)
            x1, y1 = cx + R*math.cos(ang), cy + R*math.sin(ang)
            x2, y2 = cx + r2*math.cos(ang), cy + r2*math.sin(ang)
            pygame.draw.line(self.screen, GCS_DIM,
                             (int(x1),int(y1)), (int(x2),int(y2)))
        for ang_world, label in [(-math.pi/2, "K"), (0, "D"),
                                  (math.pi/2, "G"), (math.pi, "B")]:
            lx = cx + (R - 14) * math.cos(ang_world)
            ly = cy + (R - 14) * math.sin(ang_world)
            t  = self.font_sm.render(label, True,
                                     GCS_YELLOW if label == "K" else GCS_DIM)
            self.screen.blit(t, (int(lx - t.get_width()//2),
                                  int(ly - t.get_height()//2)))
        ax = cx + int(R * 0.65 * math.cos(theta))
        ay = cy + int(R * 0.65 * math.sin(theta))
        pygame.draw.polygon(self.screen, acc, [
            (ax, ay),
            (int(cx + R*0.35*math.cos(theta + 2.5)),
             int(cy + R*0.35*math.sin(theta + 2.5))),
            (int(cx + R*0.35*math.cos(theta - 2.5)),
             int(cy + R*0.35*math.sin(theta - 2.5))),
        ])
        pygame.draw.circle(self.screen, GCS_BORDER, (cx, cy), R, 1)
        ht = self.font_sm.render(f"{math.degrees(theta)%360:.0f} deg", True, GCS_TEXT)
        self.screen.blit(ht, (cx - ht.get_width()//2, cy + R + 4))
        return cy + R + ht.get_height() + 10

    def _draw_hsi(self, cx, cy_center, theta, y_base):
        """Hava araçları için HSI göstergesi."""
        R  = 38; cy = cy_center
        acc = self._theme["accent"]
        pygame.draw.circle(self.screen, (10, 15, 32), (cx, cy), R)
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            r2  = R - (10 if deg % 90 == 0 else 6)
            x1, y1 = cx + R*math.cos(rad - math.pi/2), cy + R*math.sin(rad - math.pi/2)
            x2, y2 = cx + r2*math.cos(rad - math.pi/2), cy + r2*math.sin(rad - math.pi/2)
            pygame.draw.line(self.screen, GCS_DIM,
                             (int(x1),int(y1)), (int(x2),int(y2)))
        for deg, label, c in [(0,"K",GCS_YELLOW),(90,"D",GCS_DIM),
                               (180,"G",GCS_DIM),(270,"B",GCS_DIM)]:
            rad = math.radians(deg) - math.pi/2
            lx = cx + (R-16)*math.cos(rad)
            ly = cy + (R-16)*math.sin(rad)
            t  = self.font_sm.render(label, True, c)
            self.screen.blit(t, (int(lx-t.get_width()//2),
                                  int(ly-t.get_height()//2)))
        dial = theta - math.pi/2
        ax = cx + int(R * 0.62 * math.cos(dial))
        ay = cy + int(R * 0.62 * math.sin(dial))
        pygame.draw.polygon(self.screen, acc, [
            (ax, ay),
            (int(cx + R*0.3*math.cos(dial + 2.4)),
             int(cy + R*0.3*math.sin(dial + 2.4))),
            (int(cx + R*0.3*math.cos(dial - 2.4)),
             int(cy + R*0.3*math.sin(dial - 2.4))),
        ])
        bax = cx + int(R * 0.62 * math.cos(dial + math.pi))
        bay = cy + int(R * 0.62 * math.sin(dial + math.pi))
        pygame.draw.line(self.screen, (120,120,140), (cx,cy), (bax,bay), 1)
        pygame.draw.circle(self.screen, acc, (cx, cy), 4)
        pygame.draw.circle(self.screen, GCS_BORDER, (cx, cy), R, 1)
        ht = self.font_sm.render(f"{math.degrees(theta)%360:.0f} deg", True, GCS_TEXT)
        self.screen.blit(ht, (cx - ht.get_width()//2, cy + R + 4))
        return cy + R + ht.get_height() + 10

    def _draw_buttons(self, px):
        mx, my = pygame.mouse.get_pos()
        pause_lbl = "DURAKLAT" if not self.paused else "DEVAM ET"
        specs = [
            ("pause",  pause_lbl,   (28,48,88),(50,80,145),(80,130,205)),
            ("faster", "+ HIZLAN",  (20,45,20),(42,78,42), GCS_GREEN),
            ("slower", "- YAVASLA", (45,20,20),(80,42,42), GCS_WARN),
            ("replay", "TEKRAR",    (15,28,65),(30,52,120),(100,155,255)),
            ("menu",   "ANA MENU",  (30,14,50),(56,28,92), (155,90,210)),
        ]
        for key, label, cn, ch, brd in specs:
            btn   = self._btns[key]
            hover = btn.collidepoint(mx, my)
            pygame.draw.rect(self.screen, ch if hover else cn, btn, border_radius=6)
            pygame.draw.rect(self.screen, brd, btn, 1, border_radius=6)
            lt = self.font_md.render(label, True, WHITE)
            self.screen.blit(lt, (btn.x + (btn.w - lt.get_width()) // 2,
                                   btn.y + (btn.h - lt.get_height()) // 2))

    # ─── Yakın görünüm (zoom inset) ──────────────────────────────────

    def _draw_zoom_panel(self, robot):
        """Haritanın sol kenar boşluğuna yakınlaştırılmış araç görünümü."""
        # Sol kenar boşluğuna sığacak boyutu hesapla
        margin = self.map_x0 - 8          # kullanılabilir sol boşluk
        zm = min(self.map_w // 3, 260, margin)
        if zm < 60:
            return
        zx = (self.map_x0 - zm) // 2     # sol boşlukta yatay ortala
        zy = self.map_y0 + (self.map_h - zm) // 2   # haritayla dikey ortala

        zoom    = 3.0
        z_cell  = self.cell_px * zoom
        cs      = self.map.cell_size
        z_scale = z_cell / cs         # px / metre

        cx_w, cy_w = robot.state.x, robot.state.y
        zsurf = pygame.Surface((zm, zm))
        zsurf.fill((6, 10, 18))

        cx_z = zm // 2
        cy_z = zm // 2

        def w2z(wx, wy):
            return (int(cx_z + (wx - cx_w) * z_scale),
                    int(cy_z + (wy - cy_w) * z_scale))

        # ── Harita hücreleri ──────────────────────────────────────────
        view_m = (zm / 2) / z_scale
        z_cp   = max(int(z_cell), 2)
        r_lo = max(0, int((cy_w - view_m) / cs))
        r_hi = min(self.map.rows, int((cy_w + view_m) / cs) + 2)
        c_lo = max(0, int((cx_w - view_m) / cs))
        c_hi = min(self.map.cols, int((cx_w + view_m) / cs) + 2)

        for r in range(r_lo, r_hi):
            for c in range(c_lo, c_hi):
                px_, py_ = w2z(c * cs, r * cs)
                rect2 = pygame.Rect(px_, py_, z_cp, z_cp)
                if self.map.is_padding(r, c):
                    pygame.draw.rect(zsurf, (100, 65, 25), rect2)
                elif not self.map.is_free(r, c):
                    pygame.draw.rect(zsurf, OBSTACLE_C, rect2)
                else:
                    pygame.draw.rect(zsurf, (16, 24, 36), rect2, 1)

        # ── LiDAR ─────────────────────────────────────────────────────
        cx_lpx, cy_lpx = w2z(cx_w, cy_w)
        for pt in self._lidar_raw:
            pygame.draw.line(zsurf, (90, 22, 22),
                             (cx_lpx, cy_lpx), w2z(pt[0], pt[1]), 1)
        for pt in self._lidar_points:
            pygame.draw.circle(zsurf, (200, 60, 60), w2z(pt[0], pt[1]), 3)

        # ── Robot ─────────────────────────────────────────────────────
        rz_px  = max(int(robot.radius * z_scale), 7)
        theta  = robot.state.theta
        body_c = BODY_COLORS.get(robot.robot_type, (60, 100, 200))
        cos_t  = math.cos(theta)
        sin_t  = math.sin(theta)

        def zrot(lx, ly):
            return (int(lx * cos_t - ly * sin_t + cx_z),
                    int(lx * sin_t + ly * cos_t + cy_z))

        rtype = robot.robot_type
        if rtype in ("DifferentialDrive", "Mecanum", "Ackermann", "Quadruped"):
            bw = rz_px * 2.2; bh = rz_px * 1.5
            bp = [zrot(-bw/2, -bh/2), zrot(bw/2, -bh/2),
                  zrot( bw/2,  bh/2), zrot(-bw/2,  bh/2)]
            pygame.draw.polygon(zsurf, body_c, bp)
            pygame.draw.polygon(zsurf, WHITE, bp, 2)
            fwd = zrot(int(rz_px * 1.8), 0)
            pygame.draw.line(zsurf, WHITE, (cx_z, cy_z), fwd, 3)
            pygame.draw.circle(zsurf, GCS_YELLOW, fwd, 5)
        elif rtype in ("OmniWheel", "Hexapod", "Bipedal"):
            pygame.draw.circle(zsurf, body_c, (cx_z, cy_z), rz_px)
            pygame.draw.circle(zsurf, WHITE,  (cx_z, cy_z), rz_px, 2)
            fwd = zrot(int(rz_px * 1.4), 0)
            pygame.draw.line(zsurf, WHITE, (cx_z, cy_z), fwd, 3)
            pygame.draw.circle(zsurf, GCS_YELLOW, fwd, 5)
        elif rtype == "SnakeRobot":
            for i in range(5):
                spt = zrot(int(-rz_px * i * 0.5), 0)
                bc_ = tuple(max(body_c[j] - i * 18, 0) for j in range(3))
                pygame.draw.circle(zsurf, bc_, spt, max(rz_px - i * 2, 3))
            pygame.draw.circle(zsurf, GCS_YELLOW, zrot(rz_px, 0), 5)
        elif rtype == "Quadrotor":
            arm = rz_px * 2.8
            for i in range(4):
                aa  = theta + math.pi / 4 + i * math.pi / 2
                tip = (int(cx_z + arm * math.cos(aa)), int(cy_z + arm * math.sin(aa)))
                pygame.draw.line(zsurf, (150, 150, 150), (cx_z, cy_z), tip, 2)
                pygame.draw.circle(zsurf, body_c, tip, max(int(rz_px * 0.65), 4), 2)
            pygame.draw.circle(zsurf, body_c, (cx_z, cy_z), max(int(rz_px * 0.6), 4))
            pygame.draw.circle(zsurf, WHITE,  (cx_z, cy_z), max(int(rz_px * 0.6), 4), 1)
            pygame.draw.circle(zsurf, GCS_YELLOW, zrot(int(rz_px * 0.5), 0), 3)
        elif rtype in ("VTOL", "FixedWing"):
            fl = rz_px * 2.5; fh = rz_px * 0.5; wl = rz_px * 2.0
            for side in (-1, 1):
                wp = [zrot(int(-fl*0.1), int(side*fh*0.5)),
                      zrot(int( fl*0.2), int(side*fh*0.5)),
                      zrot(int(-fl*0.2), int(side*(fh*0.5 + wl))),
                      zrot(int(-fl*0.3), int(side*(fh*0.5 + wl)))]
                pygame.draw.polygon(zsurf, body_c, wp)
            bp2 = [zrot(int(-fl/2), int(-fh/2)), zrot(int(fl/2), int(-fh/2)),
                   zrot(int( fl/2), int( fh/2)), zrot(int(-fl/2), int( fh/2))]
            pygame.draw.polygon(zsurf, tuple(min(c+20, 255) for c in body_c), bp2)
            pygame.draw.polygon(zsurf, WHITE, bp2, 2)
            pygame.draw.circle(zsurf, GCS_YELLOW, zrot(int(fl * 0.55), 0), 5)
        else:
            pygame.draw.circle(zsurf, body_c, (cx_z, cy_z), rz_px)
            pygame.draw.circle(zsurf, WHITE,  (cx_z, cy_z), rz_px, 2)
            fwd = zrot(int(rz_px * 1.3), 0)
            pygame.draw.line(zsurf, WHITE, (cx_z, cy_z), fwd, 3)

        # ── Artı imleci ───────────────────────────────────────────────
        pygame.draw.line(zsurf, (28, 46, 68), (cx_z, 2),  (cx_z, zm - 2), 1)
        pygame.draw.line(zsurf, (28, 46, 68), (2, cy_z), (zm - 2, cy_z), 1)

        # ── Yüzeyi ekrana yapıştır + kenarlık ─────────────────────────
        self.screen.blit(zsurf, (zx, zy))
        zrect = pygame.Rect(zx, zy, zm, zm)
        acc   = self._theme["accent"]
        pygame.draw.rect(self.screen, acc, zrect, 2, border_radius=6)

        # ── Etiket ────────────────────────────────────────────────────
        lbl = self.font_sm.render(
            f" YAKIN GORUNUM  x{int(zoom)} ", True, acc)
        lb_bg = pygame.Rect(zx + 4, zy + 4,
                            lbl.get_width() + 4, lbl.get_height() + 2)
        pygame.draw.rect(self.screen, (4, 8, 15), lb_bg, border_radius=3)
        self.screen.blit(lbl, (zx + 6, zy + 5))

        # ── Koordinat ─────────────────────────────────────────────────
        coord = self.font_sm.render(
            f"({cx_w:.1f},{cy_w:.1f})", True, GCS_DIM)
        self.screen.blit(
            coord,
            (zx + zm - coord.get_width() - 4, zy + zm - coord.get_height() - 3))

    # ─── Harita yüzeyi ────────────────────────────────────────────────

    def _build_map_surface(self) -> pygame.Surface:
        surf = pygame.Surface((self.map_w, self.map_h))
        surf.fill(MAP_BG)
        cp = self.cell_px
        for r in range(self.map.rows):
            for c in range(self.map.cols):
                rect = pygame.Rect(c*cp, r*cp, cp, cp)
                if self.map.is_padding(r, c):
                    pygame.draw.rect(surf, PADDING_C, rect)
                elif not self.map.is_free(r, c):
                    pygame.draw.rect(surf, OBSTACLE_C, rect)
                else:
                    pygame.draw.rect(surf, MAP_GRID, rect, 1)
        sr, sc2 = self.map.start; gr, gc = self.map.goal
        pygame.draw.rect(surf, (30, 180, 60),
                         pygame.Rect(sc2*cp+1, sr*cp+1, cp-2, cp-2))
        pygame.draw.rect(surf, (200, 40, 40),
                         pygame.Rect(gc*cp+1,  gr*cp+1, cp-2, cp-2))
        if cp >= 10:
            fs = pygame.font.SysFont("consolas", max(cp-4, 8), bold=True)
            for lbl, rx, ry, c2 in [("S", sc2, sr, WHITE), ("G", gc, gr, WHITE)]:
                t = fs.render(lbl, True, c2)
                surf.blit(t, (rx*cp + (cp-t.get_width())//2,
                               ry*cp + (cp-t.get_height())//2))
        return surf

    # ─── Koordinat dönüşümleri ────────────────────────────────────────

    def _rc_to_px(self, r: int, c: int) -> Tuple[int, int]:
        return (self.map_x0 + c * self.cell_px + self.cell_px // 2,
                self.map_y0 + r * self.cell_px + self.cell_px // 2)

    def _world_to_px(self, x: float, y: float) -> Tuple[int, int]:
        c = x / self.map.cell_size
        r = y / self.map.cell_size
        return (int(self.map_x0 + c * self.cell_px),
                int(self.map_y0 + r * self.cell_px))
