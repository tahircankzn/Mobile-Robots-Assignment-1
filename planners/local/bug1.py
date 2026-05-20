"""
planners/local/bug1.py
Bug1 Algoritması — tam çevreleme tabanlı engel aşma.

Davranış:
    GO_TO_GOAL    : Hedefe doğru hareket et.
                    Engel tespit edilince → hit noktasını kaydet → CIRCUMNAVIGATE.
    CIRCUMNAVIGATE: Engelin tamamını çevrele.
                    Hedefe en yakın nokta (q_leave) takip edilir.
                    Hit noktasına geri dönünce → GO_TO_LEAVE.
    GO_TO_LEAVE   : Sınır boyunca q_leave noktasına git → GO_TO_GOAL.

Özellik:
    - Tamamlanma garantisi VAR (yol mevcutsa mutlaka bulunur).
    - Bug0 ve Bug2'ye göre daha yavaş (tüm engeli çevrelemek gerekir).
    - Engelin convex/concave olmasına bakılmaksızın çalışır.
"""

import numpy as np
from .base_local import BaseLocal, angle_diff


# ─── Parametre sabitleri ─────────────────────────────────────────────────────
_OBSTACLE_DIST    = 1.2
_WALL_DIST        = 0.8
_FORWARD_FOV      = np.pi / 2.5
_SIDE_FOV_CENTER  = -np.pi / 2
_SIDE_FOV_WIDTH   = np.pi / 3
_HIT_REACH_DIST   = 1.2    # (m) hit noktasına “ualaştı” eşiği (genişletildi)
_LEAVE_REACH_DIST = 1.0    # (m) leave noktasına “ualaştı” eşiği
_MIN_CIRC_STEPS   = 80     # hit noktasına geri dönmeden önce minimum çevreleme adımı
_MAX_CIRC_STEPS   = 2500   # çevreleme zaman aşımı (adım) — sonsuz döngü koruması
_MAX_LEAVE_STEPS  = 500    # GO_TO_LEAVE zaman aşımı


class Bug1(BaseLocal):
    """
    Bug1: Tam engel çevrelemesi ile en yakın ayrılma noktası bulma.

    Durum makinesi: GO_TO_GOAL → CIRCUMNAVIGATE → GO_TO_LEAVE → GO_TO_GOAL
    """

    name = "Bug1"

    def __init__(self):
        self._state       = "GO_TO_GOAL"
        self._hit_x       = 0.0
        self._hit_y       = 0.0
        self._leave_x     = 0.0
        self._leave_y     = 0.0
        self._min_dist    = float("inf")
        self._circ_steps  = 0

    def reset(self, start_x, start_y, goal_x, goal_y):
        self._state      = "GO_TO_GOAL"
        self._min_dist   = float("inf")
        self._circ_steps = 0

    # ─── Ana hesaplama ───────────────────────────────────────────────────────

    def compute(self, rx, ry, rtheta, ranges, beam_angles, gx, gy):
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
                self._min_dist   = dist_goal
                self._leave_x    = rx
                self._leave_y    = ry
                self._circ_steps = 0
                self._state      = "CIRCUMNAVIGATE"
                return 0.15, 1.3

            v     = min(1.2, dist_goal * 0.55)
            omega = np.clip(2.5 * goal_err, -1.8, 1.8)
            return v, omega

        # ── CIRCUMNAVIGATE ───────────────────────────────────────────────────
        if self._state == "CIRCUMNAVIGATE":
            self._circ_steps += 1

            # Hedefe en yakın nokta kaydı
            if dist_goal < self._min_dist:
                self._min_dist = dist_goal
                self._leave_x  = rx
                self._leave_y  = ry

            # Zaman aşımı: bu kadar adımda hit noktasına dönülemediyse GO_TO_LEAVE'e geç
            if self._circ_steps >= _MAX_CIRC_STEPS:
                self._state      = "GO_TO_LEAVE"
                self._circ_steps = 0
                return 0.3, 0.5

            # Yeterli adımdan sonra hit noktasına döndük mü?
            dist_to_hit = np.hypot(rx - self._hit_x, ry - self._hit_y)
            if self._circ_steps >= _MIN_CIRC_STEPS and dist_to_hit < _HIT_REACH_DIST:
                self._state      = "GO_TO_LEAVE"
                self._circ_steps = 0
                return 0.3, 0.5

            return self._wall_follow(min_front, right_dist)

        # ── GO_TO_LEAVE ──────────────────────────────────────────────────────
        if self._state == "GO_TO_LEAVE":
            self._circ_steps += 1

            dist_to_leave = np.hypot(rx - self._leave_x, ry - self._leave_y)
            if dist_to_leave < _LEAVE_REACH_DIST:
                # Leave noktasına ulaştık → hedefe doğru yönel
                self._state      = "GO_TO_GOAL"
                self._circ_steps = 0
                return 0.5, np.clip(2.5 * goal_err, -1.8, 1.8)

            # Zaman aşımı
            if self._circ_steps > _MAX_LEAVE_STEPS:
                self._state      = "GO_TO_GOAL"
                self._circ_steps = 0
                return 0.5, 0.0

            # Doğrudan leave noktasına yönel; önde engel varsa duvar takibine geç
            leave_angle = np.arctan2(self._leave_y - ry, self._leave_x - rx)
            leave_err   = angle_diff(leave_angle, rtheta)
            if min_front < _OBSTACLE_DIST:
                return self._wall_follow(min_front, right_dist)
            v     = min(0.8, dist_to_leave * 0.45)
            omega = np.clip(2.5 * leave_err, -1.8, 1.8)
            return v, omega

        return 0.0, 0.0

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
