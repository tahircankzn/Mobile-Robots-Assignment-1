"""
robots/quadruped.py
Quadruped (Dört Bacaklı Robot Köpek) kinematik modeli.
Boston Dynamics Spot benzeri platform.

Kontrol girişi: (vx_r, vy_r, omega)
    vx_r  : robot gövde çerçevesinde ileri/geri hız (m/s)
    vy_r  : robot gövde çerçevesinde yanal hız (m/s)  — penaltılı, max ≤ |vx_r|×0.60
    omega : açısal hız (rad/s)

Holonomik benzeri; yanal hareket fiziksel olarak mümkün fakat kısıtlıdır.
Gayt modu otomatik: walk (<0.5 m/s) / trot (0.5–1.5 m/s) / bound (>1.5 m/s).
"""

import numpy as np
from .base_robot import BaseRobot, RobotState

_LAT_RATIO = 0.60   # yanal hız = ileri hız × bu oran (max)


class Quadruped(BaseRobot):
    """4 bacaklı robot köpek — holonomik benzeri, yanal hız kısıtlı."""

    def __init__(self, x: float = 0.0, y: float = 0.0,
                 theta: float = 0.0, radius: float = 0.32):
        super().__init__(x, y, theta, radius)
        self._v_max     = 2.0       # m/s (trot hızı)
        self._omega_max = np.pi     # rad/s
        self.gait       = "walk"    # animasyon / durum bilgisi

    # ------------------------------------------------------------------

    @property
    def robot_type(self) -> str:
        return "Quadruped"

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

        # Yanal hız penaltısı: |vy_r| ≤ |vx_r| × LAT_RATIO
        v_fwd = abs(vx_r)
        vy_r  = np.clip(vy_r, -v_fwd * _LAT_RATIO, v_fwd * _LAT_RATIO)

        # Toplam hız sınırlama
        v_mag = np.hypot(vx_r, vy_r)
        if v_mag > self._v_max:
            scale = self._v_max / v_mag
            vx_r *= scale
            vy_r *= scale
            v_mag = self._v_max

        omega = np.clip(omega, -self._omega_max, self._omega_max)

        # Gayt modu güncelle
        if v_mag < 0.5:
            self.gait = "walk"
        elif v_mag < 1.5:
            self.gait = "trot"
        else:
            self.gait = "bound"

        # Gövde çerçevesinden dünya çerçevesine dönüşüm
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
