"""
robots/snake_robot.py
Yılan Robot (Snake Robot) kinematik modeli.

Kontrol: (v, omega)
    v     : baş segmenti lineer hız (m/s)
    omega : baş segmenti açısal hız (rad/s)

N=6 gövde segmenti arka arkaya kinematic-chain yaklaşımıyla takip eder.
Her segment, önceki segmente sabit LINK_LEN mesafede kalır.

Özellikleri:
    - Dar geçitlerde çok avantajlı (küçük çarpışma yarıçapı = 0.18 m)
    - Toplam gövde uzunluğu ≈ N × LINK_LEN = 6 × 0.22 = 1.32 m
    - Non-holonomic: yalnızca baş yönünde hareket
"""

import numpy as np
from .base_robot import BaseRobot, RobotState

_N_LINKS  = 6      # gövde segment sayısı (baş dahil)
_LINK_LEN = 0.22   # ardışık segmentler arası mesafe (m)


class SnakeRobot(BaseRobot):
    """
    N-eklemli seri kinematik zincir yılan robot.
    Non-holonomic: baş segmenti unicycle modelle kontrol edilir.
    """

    def __init__(self, x: float = 0.0, y: float = 0.0,
                 theta: float = 0.0, radius: float = 0.18):
        super().__init__(x, y, theta, radius)
        self._v_max     = 1.5
        self._omega_max = 2.0

        # Segment pozisyonları: shape (N, 2), index 0 = baş
        self.segments = np.array(
            [[x - i * _LINK_LEN * np.cos(theta),
              y - i * _LINK_LEN * np.sin(theta)]
             for i in range(_N_LINKS)], dtype=float
        )

    # ------------------------------------------------------------------

    @property
    def robot_type(self) -> str:
        return "SnakeRobot"

    @property
    def max_linear_speed(self) -> float:
        return self._v_max

    @property
    def max_angular_speed(self) -> float:
        return self._omega_max

    # ------------------------------------------------------------------

    def reset(self, x: float, y: float, theta: float = 0.0):
        super().reset(x, y, theta)
        self.segments = np.array(
            [[x - i * _LINK_LEN * np.cos(theta),
              y - i * _LINK_LEN * np.sin(theta)]
             for i in range(_N_LINKS)], dtype=float
        )

    def kinematic_step(self, control_input, dt: float) -> RobotState:
        """
        Args:
            control_input: (v, omega) veya dict
        """
        v, omega = self._parse_input(control_input)
        v     = np.clip(v,    -self._v_max,     self._v_max)
        omega = np.clip(omega, -self._omega_max, self._omega_max)

        # Baş segmenti unicycle kinematik
        new_theta = self.wrap_angle(self.state.theta + omega * dt)
        new_x = self.state.x + v * np.cos(new_theta) * dt
        new_y = self.state.y + v * np.sin(new_theta) * dt

        # Gövde zinciri: her segment öncekini sabit link_len mesafede izler
        new_segs = self.segments.copy()
        new_segs[0] = [new_x, new_y]
        for i in range(1, _N_LINKS):
            vec  = new_segs[i - 1] - new_segs[i]
            dist = np.linalg.norm(vec)
            if dist > 1e-6:
                new_segs[i] = new_segs[i - 1] - (vec / dist) * _LINK_LEN
        self.segments = new_segs

        self.state.x     = float(new_segs[0][0])
        self.state.y     = float(new_segs[0][1])
        self.state.theta = new_theta
        self.state.vx    = v * np.cos(new_theta)
        self.state.vy    = v * np.sin(new_theta)
        self.state.omega = omega

        self._record()
        return self.state

    # ------------------------------------------------------------------

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl['v'], ctrl['omega']
        return float(ctrl[0]), float(ctrl[1])
