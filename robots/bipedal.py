"""
robots/bipedal.py
Bipedal (İki Bacaklı İnsansı Robot) kinematik modeli.
Atlas / ASIMO benzeri platform — basitleştirilmiş 2D kinematik.

Kontrol: (v, omega)
    v     : lineer hız (m/s) — diferansiyel sürüş benzeri
    omega : açısal hız (rad/s)

Özellikler:
    - Dönüş sırasında ileri hız otomatik azalır (gerçekçi insansı yürüyüş kısıtı)
    - Adım birikimi animasyon fazını takip eder
    - Non-holonomic
"""

import numpy as np
from .base_robot import BaseRobot, RobotState

_STEP_DIST = 0.20   # animasyon adım mesafesi (m)


class Bipedal(BaseRobot):
    """
    İki bacaklı insansı robot — non-holonomic, adım tabanlı ilerleme.
    Dönüş sırasında hız düşer (human-like gait constraint).
    """

    def __init__(self, x: float = 0.0, y: float = 0.0,
                 theta: float = 0.0, radius: float = 0.25):
        super().__init__(x, y, theta, radius)
        self._v_max     = 1.6
        self._omega_max = 1.2 * np.pi

        self._dist_acc  = 0.0   # adım birikimi (m)
        self.step_phase = 0     # 0 = sol adım, 1 = sağ adım

    # ------------------------------------------------------------------

    @property
    def robot_type(self) -> str:
        return "Bipedal"

    @property
    def max_linear_speed(self) -> float:
        return self._v_max

    @property
    def max_angular_speed(self) -> float:
        return self._omega_max

    # ------------------------------------------------------------------

    def reset(self, x: float, y: float, theta: float = 0.0):
        super().reset(x, y, theta)
        self._dist_acc  = 0.0
        self.step_phase = 0

    def kinematic_step(self, control_input, dt: float) -> RobotState:
        """
        Args:
            control_input: (v, omega) veya dict
        """
        v, omega = self._parse_input(control_input)

        # Dönüş sırasında hız azalt (insansı yürüyüş kısıtı)
        turn_factor = max(0.35, 1.0 - abs(omega) / self._omega_max * 0.65)
        v     = np.clip(v,     0.0,               self._v_max * turn_factor)
        omega = np.clip(omega, -self._omega_max,  self._omega_max)

        theta = self.state.theta
        self.state.x     += v * np.cos(theta) * dt
        self.state.y     += v * np.sin(theta) * dt
        self.state.theta  = self.wrap_angle(theta + omega * dt)
        self.state.vx     = v * np.cos(self.state.theta)
        self.state.vy     = v * np.sin(self.state.theta)
        self.state.omega  = omega

        # Adım fazı güncellemesi
        self._dist_acc += v * dt
        if self._dist_acc >= _STEP_DIST:
            self._dist_acc  -= _STEP_DIST
            self.step_phase  = 1 - self.step_phase

        self._record()
        return self.state

    # ------------------------------------------------------------------

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl['v'], ctrl['omega']
        return float(ctrl[0]), float(ctrl[1])
