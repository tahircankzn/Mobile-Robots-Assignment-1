"""
robots/ackermann.py
Ackermann (araba tipi) kinematik modeli.

Kontrol girişi: (v, delta)
    v     : lineer hız (m/s)
    delta : ön tekerlek direksiyon açısı (radyan)

Bicycle model kinematik denklemleri:
    x'     = v * cos(θ)
    y'     = v * sin(θ)
    theta' = (v / L) * tan(delta)

Non-holonomic: yana kayamaz, minimum dönüş yarıçapı kısıtı vardır.
"""

import numpy as np
from .base_robot import BaseRobot, RobotState


class Ackermann(BaseRobot):
    """
    Ackermann / araba tipi robot (bicycle model).
    Non-holonomic.

    Args:
        wheel_base (L):   Ön-arka aks mesafesi (m).
        max_steer (delta_max): Maksimum direksiyon açısı (rad).
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0,
                 wheel_base=0.8, max_steer=np.radians(35),
                 width=0.4, length=0.8, radius=0.45):
        super().__init__(x, y, theta, radius)
        self.L = wheel_base
        self.delta_max = max_steer
        self.width = width
        self.length = length
        self._v_max = 3.0
        self._omega_max = self._v_max / self.L * np.tan(max_steer)

    # ------------------------------------------------------------------

    @property
    def robot_type(self) -> str:
        return "Ackermann"

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
            control_input: (v, delta) veya dict
        """
        v, delta = self._parse_input(control_input)

        v = np.clip(v, -self._v_max, self._v_max)
        delta = np.clip(delta, -self.delta_max, self.delta_max)

        theta = self.state.theta

        # Bicycle model
        self.state.x     += v * np.cos(theta) * dt
        self.state.y     += v * np.sin(theta) * dt
        dtheta = (v / self.L) * np.tan(delta)
        self.state.theta  = self.wrap_angle(theta + dtheta * dt)
        self.state.vx     = v * np.cos(self.state.theta)
        self.state.vy     = v * np.sin(self.state.theta)
        self.state.omega  = dtheta

        self._record()
        return self.state

    def min_turn_radius(self) -> float:
        """Minimum dönüş yarıçapı (metre)."""
        return self.L / np.tan(self.delta_max)

    # ------------------------------------------------------------------

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl['v'], ctrl['delta']
        return float(ctrl[0]), float(ctrl[1])
