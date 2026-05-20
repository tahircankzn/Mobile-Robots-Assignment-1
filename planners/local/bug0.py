"""
planners/local/bug0.py
Bug0 Algoritması — en basit reaktif navigasyon.

Davranış (Sağ-El Kuralı):
    GO_TO_GOAL      : Hedefe doğru düz git.
                      Ön sektörde engel tespit edilirse → FOLLOW_BOUNDARY.
    FOLLOW_BOUNDARY : Sağ duvarı takip et (engel robotun sağ tarafında kalır).
                      Hedef yönü açılınca → GO_TO_GOAL.

Özellik:
    - En basit Bug varyantı. Bellek gerektirmez.
    - Tamamlanma garantisi YOKTUR (tüm engel konfigürasyonlarında döngüye girebilir).
    - Düz koridorlar ve basit engeller için yeterli.
"""

import numpy as np
from .base_local import BaseLocal, angle_diff


# ─── Parametre sabitleri ─────────────────────────────────────────────────────
_OBSTACLE_DIST    = 1.2    # (m) ön sektörde engel eşiği
_WALL_DIST        = 0.8    # (m) sağ duvar hedef mesafesi
_FORWARD_FOV      = np.pi / 2.5   # ön sektör açısı (yaklaşık ±72°)
_SIDE_FOV_CENTER  = -np.pi / 2    # sağ taraf merkezi (−0° = sağ)
_SIDE_FOV_WIDTH   = np.pi / 3     # sağ taraf sektör genişliği (±30°)
_MIN_BDRY_STEPS   = 30            # hedef kontrolü öncesi minimum sınır adımı
_MAX_BDRY_STEPS   = 2500          # zaman aşımı koruması (adım)
_GOAL_BEAM_TOL    = np.pi / 8     # hedef ışını kontrolü toleransı (rad)


class Bug0(BaseLocal):
    """
    Bug0: Reaktif engel sınırı takibi (sağ-el kuralı).

    Durum makinesi: GO_TO_GOAL ↔ FOLLOW_BOUNDARY
    """

    name = "Bug0"

    def __init__(self):
        self._state = "GO_TO_GOAL"
        self._bdry_steps = 0

    def reset(self, start_x, start_y, goal_x, goal_y):
        self._state      = "GO_TO_GOAL"
        self._bdry_steps = 0

    # ─── Ana hesaplama ───────────────────────────────────────────────────────

    def compute(self, rx, ry, rtheta, ranges, beam_angles, gx, gy):
        goal_angle = np.arctan2(gy - ry, gx - rx)
        goal_err   = angle_diff(goal_angle, rtheta)   # [-π, π]
        dist_goal  = np.hypot(gx - rx, gy - ry)

        # Ön sektör (± _FORWARD_FOV/2) minimum mesafe
        fwd_mask  = np.abs(beam_angles) < _FORWARD_FOV / 2
        min_front = float(np.min(ranges[fwd_mask])) if fwd_mask.any() else 99.0

        # Sağ taraf minimum mesafe
        side_mask  = np.abs(beam_angles - _SIDE_FOV_CENTER) < _SIDE_FOV_WIDTH / 2
        right_dist = float(np.min(ranges[side_mask])) if side_mask.any() else 99.0

        # ── GO_TO_GOAL ───────────────────────────────────────────────────────
        if self._state == "GO_TO_GOAL":
            if min_front < _OBSTACLE_DIST:
                # Engel tespit edildi → sola dön (engel sağa düşsün), sınır moduna geç
                self._state      = "FOLLOW_BOUNDARY"
                self._bdry_steps = 0
                return 0.15, 1.3

            v     = min(1.2, dist_goal * 0.55)
            omega = np.clip(2.5 * goal_err, -1.8, 1.8)
            return v, omega

        # ── FOLLOW_BOUNDARY ──────────────────────────────────────────────────
        self._bdry_steps += 1

        # Yeterli adım geçtikten sonra hedef yönünü kontrol et
        if self._bdry_steps >= _MIN_BDRY_STEPS and min_front >= _OBSTACLE_DIST:
            goal_rel = angle_diff(goal_angle, rtheta)
            beam_diffs = np.abs(beam_angles - goal_rel)
            beam_idx   = int(np.argmin(beam_diffs))
            goal_beam_range = ranges[beam_idx]
            # Hedef yönünde yeterli açıklık varsa hedefe dön
            if goal_beam_range > _OBSTACLE_DIST * 1.1:
                self._state      = "GO_TO_GOAL"
                self._bdry_steps = 0
                return 0.5, np.clip(2.5 * goal_err, -1.8, 1.8)

        # Zaman aşımı — döngü kırıcı
        if self._bdry_steps >= _MAX_BDRY_STEPS:
            self._state      = "GO_TO_GOAL"
            self._bdry_steps = 0
            return 0.5, np.clip(2.5 * goal_err, -1.8, 1.8)

        return self._wall_follow(min_front, right_dist)

    # ─── Sağ-duvar takip kontrolcüsü ────────────────────────────────────────

    def _wall_follow(self, min_front, right_dist):
        """
        Sağ-el duvar takibi:
            Sağda duvar yok (açık alan) → hafif sağa kayarak duvar ara.
            right_dist > _WALL_DIST → sağa dön (negatif omega)
            right_dist < _WALL_DIST → sola dön (pozitif omega)
        """
        if min_front < _OBSTACLE_DIST * 0.65:
            # Önde çok yakın engel → yerinde sola dön
            return 0.10, 1.5

        if right_dist > _WALL_DIST * 3.5:
            # Sağda duvar yok → hafif sağa kayarak duvar bul
            # (eski kod burada -1.5 rad/s uyguluyordu → sarmal!)
            return 0.5, -0.4

        wall_err = right_dist - _WALL_DIST   # + → uzakta (sağa dön)  - → yakında (sola dön)
        v        = 0.5
        omega    = np.clip(-1.5 * wall_err, -1.5, 1.5)
        return v, omega
