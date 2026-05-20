"""
planners/local/bug2.py
Bug2 Algoritması — M-line tabanlı engel aşma.

Temel Kavram:
    M-line: Başlangıç noktasından hedefe uzanan düz çizgi.
    Hit noktası: Engele ilk temas noktası.
    Leave noktası: M-line üzerinde hit noktasından daha yakın olan nokta.

Davranış:
    GO_TO_GOAL      : Hedefe doğru M-line yönünde hareket et.
                      Engel varsa → hit noktasını kaydet → FOLLOW_BOUNDARY.
    FOLLOW_BOUNDARY : Engel sınırını takip et.
                      M-line'a dönüldüğünde ve hedefe hit'ten daha yakınsa → GO_TO_GOAL.

Özellik:
    - Tamamlanma garantisi VAR.
    - Bug1'den genellikle daha hızlı (tüm engeli çevrelemek gerekmez).
    - Bazı engel konfigürasyonlarında Bug1'den yavaş olabilir.
"""

import numpy as np
from .base_local import BaseLocal, angle_diff


# ─── Parametre sabitleri ─────────────────────────────────────────────────────
_OBSTACLE_DIST    = 1.2
_WALL_DIST        = 0.8
_FORWARD_FOV      = np.pi / 2.5
_SIDE_FOV_CENTER  = -np.pi / 2
_SIDE_FOV_WIDTH   = np.pi / 3
_MLINE_THRESH     = 0.65   # (m) M-line üzerinde sayma eşiği
_MIN_BDRY_STEPS   = 40     # M-line kontrolü öncesi minimum sınır adımı
_MAX_BDRY_STEPS   = 2500   # zaman aşımı koruması (adım)
_CLOSER_MARGIN    = 0.35   # (m) hit noktasından daha yakın olma marjı


class Bug2(BaseLocal):
    """
    Bug2: M-line tabanlı reaktif engel aşma.

    Durum makinesi: GO_TO_GOAL ↔ FOLLOW_BOUNDARY
    """

    name = "Bug2"

    def __init__(self):
        self._state     = "GO_TO_GOAL"
        self._hit_x     = 0.0
        self._hit_y     = 0.0
        self._hit_dist  = float("inf")  # hit anındaki hedef mesafesi
        self._sx        = 0.0           # başlangıç koordinatı (M-line için)
        self._sy        = 0.0
        self._gx        = 0.0           # hedef koordinatı (M-line için)
        self._gy        = 0.0
        self._bdry_steps = 0

    def reset(self, start_x, start_y, goal_x, goal_y):
        self._state      = "GO_TO_GOAL"
        self._sx, self._sy = start_x, start_y
        self._gx, self._gy = goal_x, goal_y
        self._hit_dist   = float("inf")
        self._bdry_steps = 0

    # ─── Geometri yardımcıları ───────────────────────────────────────────────

    def _dist_to_mline(self, rx, ry):
        """Robotun mevcut konumunun M-line'a (başlangıç → hedef) dik mesafesi."""
        dx = self._gx - self._sx
        dy = self._gy - self._sy
        line_len = np.hypot(dx, dy)
        if line_len < 1e-6:
            return np.hypot(rx - self._sx, ry - self._sy)
        # ||(P-A) × (B-A)|| / ||B-A||
        px, py = rx - self._sx, ry - self._sy
        cross  = abs(px * dy - py * dx)
        return cross / line_len

    # ─── Ana hesaplama ───────────────────────────────────────────────────────

    def compute(self, rx, ry, rtheta, ranges, beam_angles, gx, gy):
        # M-line hedef koordinatını taze tut
        self._gx, self._gy = gx, gy

        goal_angle = np.arctan2(gy - ry, gx - rx)
        goal_err   = angle_diff(goal_angle, rtheta)
        dist_goal  = np.hypot(gx - rx, gy - ry)

        fwd_mask  = np.abs(beam_angles) < _FORWARD_FOV / 2
        min_front = float(np.min(ranges[fwd_mask])) if fwd_mask.any() else 99.0

        side_mask  = np.abs(beam_angles - _SIDE_FOV_CENTER) < _SIDE_FOV_WIDTH / 2
        right_dist = float(np.min(ranges[side_mask])) if side_mask.any() else 99.0

        # ── GO_TO_GOAL ───────────────────────────────────────────────────────
        if self._state == "GO_TO_GOAL":
            if min_front < _OBSTACLE_DIST:
                self._hit_x      = rx
                self._hit_y      = ry
                self._hit_dist   = dist_goal
                self._bdry_steps = 0
                self._state      = "FOLLOW_BOUNDARY"
                return 0.15, 1.3

            v     = min(1.2, dist_goal * 0.55)
            omega = np.clip(2.5 * goal_err, -1.8, 1.8)
            return v, omega

        # ── FOLLOW_BOUNDARY ──────────────────────────────────────────────────
        self._bdry_steps += 1

        # M-line üzerinde miyiz ve hedefe hit noktasından daha yakın mıyız?
        mline_dist = self._dist_to_mline(rx, ry)
        on_mline   = mline_dist < _MLINE_THRESH
        closer     = dist_goal < self._hit_dist - _CLOSER_MARGIN

        if self._bdry_steps >= _MIN_BDRY_STEPS and on_mline and closer:
            # Leave noktasına ulaştık → hedefe dön
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
        if min_front < _OBSTACLE_DIST * 0.65:
            return 0.10, 1.5

        if right_dist > _WALL_DIST * 3.5:
            # Sağda duvar yok → hafif sağa kayarak duvar ara (sarmal hatası önlemi)
            return 0.5, -0.4

        wall_err = right_dist - _WALL_DIST
        v        = 0.5
        omega    = np.clip(-1.5 * wall_err, -1.5, 1.5)
        return v, omega
