"""
robots/hexapod.py
Hexapod (Altı Bacaklı Böcek Robot) kinematik modeli.

Kontrol girişi: (vx_r, vy_r, omega)
    vx_r  : robot gövde çerçevesinde ileri/geri hız (m/s)
    vy_r  : robot gövde çerçevesinde yanal hız (m/s)
    omega : açısal hız (rad/s)

Tripod gayt: her adımda 3 bacak havada, 3 bacak yerde — statik kararlılık.
Gerçek holonomik: tüm yönlere eşit hız kapasitesi.
Maksimum hız düşük (v_max = 1.2 m/s) — yavaş fakat son derece kararlı.
"""

import numpy as np
from .base_robot import BaseRobot, RobotState


class Hexapod(BaseRobot):
    """6 bacaklı holonomik böcek robot — tripod gayt."""

    def __init__(self, x: float = 0.0, y: float = 0.0,
                 theta: float = 0.0, radius: float = 0.35):
        super().__init__(x, y, theta, radius)
        self._v_max     = 1.2
        self._omega_max = 1.5 * np.pi

    # ------------------------------------------------------------------

    @property
    def robot_type(self) -> str:
        return "Hexapod"

    @property
    def max_linear_speed(self) -> float:
        return self._v_max

    @property
    def max_angular_speed(self) -> float:
        return self._omega_max

    # ------------------------------------------------------------------

    def kinematic_step(self, control_input, dt: float) -> RobotState:
        """
        Args:
            control_input: (vx_r, vy_r, omega) veya dict
        """
        vx_r, vy_r, omega = self._parse_input(control_input)

        v_mag = np.hypot(vx_r, vy_r)
        if v_mag > self._v_max:
            s    = self._v_max / v_mag
            vx_r *= s
            vy_r *= s

        omega = np.clip(omega, -self._omega_max, self._omega_max)

        # Gövde çerçevesinden dünya çerçevesine
        theta = self.state.theta
        vx_w  = vx_r * np.cos(theta) - vy_r * np.sin(theta)
        vy_w  = vx_r * np.sin(theta) + vy_r * np.cos(theta)

        self.state.x     += vx_w * dt
        self.state.y     += vy_w * dt
        self.state.theta  = self.wrap_angle(theta + omega * dt)
        self.state.vx     = vx_w
        self.state.vy     = vy_w
        self.state.omega  = omega

        self._record()
        return self.state

    # ------------------------------------------------------------------

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl['vx'], ctrl['vy'], ctrl['omega']
        return float(ctrl[0]), float(ctrl[1]), float(ctrl[2])
