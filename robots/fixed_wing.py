"""
robots/fixed_wing.py
Sabit Kanatlı Uçak kinematik modeli.

Bicycle / Ackermann benzeri kinematik:
    x"     = v * cos(theta)
    y"     = v * sin(theta)
    theta" = (v / L) * tan(delta)

Non-holonomic: ileri hareket zorunludur, yana kayış yoktur.
Minimum dönüş yarıçapı kısıtı mevcut.
"""

import numpy as np
from .base_robot import BaseRobot, RobotState


class FixedWing(BaseRobot):
    """
    Sabit kanatlı uçak — Ackermann benzeri kinematik.
    Non-holonomik; minimum dönüş yarıçapı kısıtı var.
    Yüksek hız, büyük dönüş yarıçapı.

    Args:
        wheel_base : eşdeğer dingil mesafesi (m)
        max_bank   : maksimum banka açısı → max direksiyon (rad)
        wingspan   : kanat açıklığı (görsel, m)
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0,
                 wheel_base=1.0, max_bank=None,
                 wingspan=0.8, radius=0.5):
        super().__init__(x, y, theta, radius)
        if max_bank is None:
            max_bank = float(np.radians(25))
        self.L        = wheel_base
        self.bank_max = max_bank
        self.wingspan = wingspan
        self._v_max   = 6.0
        self._omega_max = self._v_max / self.L * np.tan(max_bank)

    @property
    def robot_type(self): return "FixedWing"

    @property
    def max_linear_speed(self): return self._v_max

    @property
    def max_angular_speed(self): return self._omega_max

    def kinematic_step(self, control_input, dt):
        v, delta = self._parse_input(control_input)
        v     = float(np.clip(v, 0.0, self._v_max))
        delta = float(np.clip(delta, -self.bank_max, self.bank_max))
        theta = self.state.theta
        self.state.x     += v * np.cos(theta) * dt
        self.state.y     += v * np.sin(theta) * dt
        dtheta = (v / self.L) * np.tan(delta) if v > 0.01 else 0.0
        self.state.theta  = self.wrap_angle(theta + dtheta * dt)
        self.state.vx     = v * np.cos(self.state.theta)
        self.state.vy     = v * np.sin(self.state.theta)
        self.state.omega  = dtheta
        self._record()
        return self.state

    def min_turn_radius(self):
        return self.L / np.tan(self.bank_max)

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl["v"], ctrl["delta"]
        return float(ctrl[0]), float(ctrl[1])