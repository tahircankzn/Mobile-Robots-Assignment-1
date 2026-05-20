"""
robots/vtol.py
VTOL (Dikey niş-Kalkış / Vertical Take-Off and Landing) kinematik modeli.

Hover modu (varsayılan): kuadrotor benzeri holonomik kinematik.
Uçuş modu: sabit kanat benzeri Ackermann kinematik (daha hızlı).
Kontrol girişi: (vx, vy, omega) — dünya koordinatlarında hız + yaw rate.
"""

import numpy as np
from .base_robot import BaseRobot, RobotState


class VTOL(BaseRobot):
    """
    VTOL (Vertical Take-Off and Landing) robot.
    Hover modunda holonomik (kuadrotor benzeri).

    Özellikler:
        wing_span : kanat açıklığı (görsel, m)
        v_max     : maksimum hız (m/s)
        omega_max : maksimum dönüş hızı (rad/s)
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0,
                 wing_span=0.6, radius=0.4):
        super().__init__(x, y, theta, radius)
        self.wing_span  = wing_span
        self._v_max     = 5.0
        self._omega_max = 2.5

    @property
    def robot_type(self): return "VTOL"

    @property
    def max_linear_speed(self): return self._v_max

    @property
    def max_angular_speed(self): return self._omega_max

    def kinematic_step(self, control_input, dt):
        vx, vy, omega = self._parse_input(control_input)
        speed = np.hypot(vx, vy)
        if speed > self._v_max:
            s = self._v_max / speed
            vx, vy = vx * s, vy * s
        omega = float(np.clip(omega, -self._omega_max, self._omega_max))
        self.state.x     += vx * dt
        self.state.y     += vy * dt
        self.state.theta  = self.wrap_angle(self.state.theta + omega * dt)
        self.state.vx, self.state.vy, self.state.omega = vx, vy, omega
        self._record()
        return self.state

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl["vx"], ctrl["vy"], ctrl["omega"]
        c = list(ctrl)
        if len(c) == 2:
            v, omega = float(c[0]), float(c[1])
            return v, 0.0, omega
        return float(c[0]), float(c[1]), float(c[2])