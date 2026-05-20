"""
planners/local/potential_fields.py
Yapay Potansiyel Alanlar (Artificial Potential Fields — APF) algoritması.

Temel Kavram:
    • Çekici potansiyel (attractive): Hedef, robotu kendine çeker.
    • İtici potansiyel (repulsive):   LiDAR ile algılanan engeller robotu iter.
    • Net kuvvet vektörü → hareket yönü ve hız büyüklüğü.

Kuvvet denklemleri:
    F_att = k_att * (goal - pos)                       [lineer, doymuş]
    F_rep = k_rep * (1/d - 1/d0) / d²  * (-obs_dir)  [d < d0 için, sektör başına]
    F_tang= k_tang * (90° to obstacle)                [yerel minimum kaçışı]

Parametreler (50×50 grid, cell_size=0.5m, engel aralıkları ~2–6m için ayarlı):
    D0=1.8m → engellerin 3.6 hücre etki yarıçapı.
    Sektör tabanlı repülsiyon: aynı engele çarpan onlarca beam'in
    toplam kuvveti şişirmesini önler.
    Geçmişe dayalı local-minimum tespiti + tangential kaçış.
"""

import numpy as np
from .base_local import BaseLocal, angle_diff


# ─── Parametre sabitleri (bu harita için kalibre edilmiş) ────────────────────
_K_ATT          = 1.5      # çekim katsayısı
_K_REP          = 0.50     # itme katsayısı — SEKTÖR BAŞINA (eski: beam başına → çok büyük)
_D0             = 1.8      # itici kuvvet etki mesafesi (m); haritada min gap ~2m
_D_GOAL_SAT     = 4.0      # çekici doyum mesafesi (m); uzaktayken sabit büyüklük
_MAX_V          = 1.1      # maksimum doğrusal hız (m/s)
_MAX_OMEGA      = 1.8      # maksimum açısal hız (rad/s)
_N_SECTORS      = 12       # repülsiyon sektör sayısı (30° aralık)
_MIN_CLEARANCE  = 0.12     # sıfır bölüm koruması (m)

# Tangential kaçış (lokal minimum defleksiyon kuvveti)
_K_TANG         = 1.2      # tangential kuvvet katsayısı
_TANG_THRESHOLD = 1.0      # net kuvvet bu değerin altındaysa tangential ekle

# İlerleme tabanlı sıkışma tespiti ve kaçış
_LM_INTERVAL    = 30       # her N adımda ilerleme kontrol et
_LM_MIN_PROG    = 0.30     # N adımda beklenen minimum ilerleme (m)
_ESCAPE_STEPS   = 60       # kaçış manevra uzunluğu (adım)


class PotentialFields(BaseLocal):
    """
    APF+: Sektör tabanlı repülsiyon + tangential kaçış + sıkışma kaçış manevra.

    Durum: yalnızca ilerleme takibi için küçük bir geçmiş tutar.
    """

    name = "PotentialFields"

    def __init__(self):
        self._step          = 0
        self._last_chk_x    = None
        self._last_chk_y    = None
        self._escape_cnt    = 0
        self._escape_omega  = 1.3     # +1.3 = sola, -1.3 = sağa

    def reset(self, start_x, start_y, goal_x, goal_y):
        self._step         = 0
        self._last_chk_x   = start_x
        self._last_chk_y   = start_y
        self._escape_cnt   = 0
        self._escape_omega = 1.3

    # ─── Ana hesaplama ───────────────────────────────────────────────────────

    def compute(self, rx, ry, rtheta, ranges, beam_angles, gx, gy):
        self._step += 1

        # ── Sıkışma tespiti (her _LM_INTERVAL adımda) ────────────────────────
        if self._step % _LM_INTERVAL == 0:
            if self._last_chk_x is not None:
                progress = np.hypot(rx - self._last_chk_x,
                                    ry - self._last_chk_y)
                if progress < _LM_MIN_PROG and self._escape_cnt == 0:
                    # En açık tarafa kaçış yönünü belirle
                    left_avg  = float(np.mean(ranges[beam_angles > 0.1]))
                    right_avg = float(np.mean(ranges[beam_angles < -0.1]))
                    self._escape_omega = 1.3 if left_avg >= right_avg else -1.3
                    self._escape_cnt   = _ESCAPE_STEPS
            self._last_chk_x = rx
            self._last_chk_y = ry

        # ── Kaçış manevrası aktifse ───────────────────────────────────────────
        if self._escape_cnt > 0:
            self._escape_cnt -= 1
            # Her 20 adımda bir yönü tersle (zigzag ile çıkış)
            if self._escape_cnt % 20 == 0:
                self._escape_omega *= -1
            return 0.35, self._escape_omega

        # ── Hedef mesafesi ────────────────────────────────────────────────────
        dx_g   = gx - rx
        dy_g   = gy - ry
        dist_g = np.hypot(dx_g, dy_g)
        if dist_g < 1e-6:
            return 0.0, 0.0

        # ── Çekici kuvvet ─────────────────────────────────────────────────────
        if dist_g <= _D_GOAL_SAT:
            fax = _K_ATT * dx_g
            fay = _K_ATT * dy_g
        else:
            fax = _K_ATT * _D_GOAL_SAT * (dx_g / dist_g)
            fay = _K_ATT * _D_GOAL_SAT * (dy_g / dist_g)

        # ── Sektör tabanlı itici kuvvet ───────────────────────────────────────
        # Her 30°'lik sektörde min range → tek katkı (beam yoğunluğuna bağımsız)
        frx, fry   = 0.0, 0.0
        sec_size   = 2.0 * np.pi / _N_SECTORS

        for s in range(_N_SECTORS):
            s_lo  = -np.pi + s * sec_size
            s_hi  = s_lo + sec_size
            mask  = (beam_angles >= s_lo) & (beam_angles < s_hi)
            if not mask.any():
                continue
            d = float(np.min(ranges[mask]))
            if d >= _D0:
                continue
            d_s       = max(d, _MIN_CLEARANCE)
            magnitude = _K_REP * (1.0 / d_s - 1.0 / _D0) / (d_s ** 2)
            # Sektör merkez açısı → dünya çerçevesi → engelden uzağa doğru
            s_center  = s_lo + sec_size * 0.5
            world_ang = rtheta + s_center
            frx      -= magnitude * np.cos(world_ang)
            fry      -= magnitude * np.sin(world_ang)

        # ── Net kuvvet ────────────────────────────────────────────────────────
        fx = fax + frx
        fy = fay + fry
        force_mag = np.hypot(fx, fy)

        # ── Tangential kaçış kuvveti (yerel minimum defleksiyon) ──────────────
        # Kuvvet zayıfsa en yakın engele dik yön ekle → çukurdan çıkış
        if force_mag < _TANG_THRESHOLD:
            min_idx  = int(np.argmin(ranges))
            obs_dir  = rtheta + float(beam_angles[min_idx])
            # İki dik seçenek — hedefe daha yakın yönü seç
            goal_dir = np.arctan2(dy_g, dx_g)
            tang_a   = obs_dir + np.pi / 2
            tang_b   = obs_dir - np.pi / 2
            best     = tang_a if (np.cos(tang_a - goal_dir) >=
                                   np.cos(tang_b - goal_dir)) else tang_b
            fx      += _K_TANG * np.cos(best)
            fy      += _K_TANG * np.sin(best)
            force_mag = np.hypot(fx, fy)

        if force_mag < 1e-6:
            return 0.15, 1.3   # son çare: yerinde dön

        # ── Kuvvet → (v, omega) ───────────────────────────────────────────────
        desired_angle = np.arctan2(fy, fx)
        theta_err     = angle_diff(desired_angle, rtheta)

        # Hız: hedef yönüne olan açı farkına göre ölçeklenir
        #   θ_err = 0   → cos = 1.0  → tam hız
        #   θ_err = 90° → cos = 0.0  → sıfır ileri hız (sadece dönüş)
        #   θ_err > 90° → negatif → klamp ile 0
        v_scale = max(0.0, np.cos(theta_err))
        v       = np.clip(force_mag * 0.28 * v_scale, 0.0, _MAX_V)
        omega   = np.clip(2.5 * theta_err, -_MAX_OMEGA, _MAX_OMEGA)

        return v, omega

