"""
robots/mecanum.py
Mecanum tekerlekli robot kinematik modeli.

Kontrol girişi: (vx, vy, omega)
    vx    : robot çerçevesinde x hızı (m/s) — ileri/geri
    vy    : robot çerçevesinde y hızı (m/s) — yanal kayma
    omega : açısal hız (rad/s)

4 tekerlek hızı (FL, FR, RL, RR) — 45° rulolar:
    v_FL = (1/r) * ( vx - vy - (Lx+Ly)*omega )
    v_FR = (1/r) * ( vx + vy + (Lx+Ly)*omega )
    v_RL = (1/r) * ( vx + vy - (Lx+Ly)*omega )
    v_RR = (1/r) * ( vx - vy + (Lx+Ly)*omega )

Holonomic: tam serbestlik — ileri, yanal ve dönme aynı anda.
"""

import numpy as np
from .base_robot import BaseRobot, RobotState


class Mecanum(BaseRobot):
    """
    4 mecanum tekerlekli holonomic robot.

    Args:
        wheel_radius (r): Tekerlek yarıçapı (m).
        lx: Merkez-tekerlek x yarı mesafesi (m).
        ly: Merkez-tekerlek y yarı mesafesi (m).
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0,
                 wheel_radius=0.05, lx=0.15, ly=0.15, radius=0.3):
        super().__init__(x, y, theta, radius)
        self.r = wheel_radius
        self.lx = lx
        self.ly = ly
        self._v_max = 2.0
        self._omega_max = 2 * np.pi

    # ------------------------------------------------------------------

    @property
    def robot_type(self) -> str:
        return "Mecanum"

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
            control_input: (vx_robot, vy_robot, omega) veya dict
        """
        vx_r, vy_r, omega = self._parse_input(control_input)

        v_mag = np.hypot(vx_r, vy_r)
        if v_mag > self._v_max:
            scale = self._v_max / v_mag
            vx_r *= scale
            vy_r *= scale
        omega = np.clip(omega, -self._omega_max, self._omega_max)

        theta = self.state.theta

        # Robot → Dünya
        vx_w = vx_r * np.cos(theta) - vy_r * np.sin(theta)
        vy_w = vx_r * np.sin(theta) + vy_r * np.cos(theta)

        self.state.x     += vx_w * dt
        self.state.y     += vy_w * dt
        self.state.theta  = self.wrap_angle(theta + omega * dt)
        self.state.vx     = vx_w
        self.state.vy     = vy_w
        self.state.omega  = omega

        self._record()
        return self.state

    def wheel_speeds(self, vx_r: float, vy_r: float, omega: float):
        """
        (vx_robot, vy_robot, omega) → (v_FL, v_FR, v_RL, v_RR) rad/s.
        FL=Ön Sol, FR=Ön Sağ, RL=Arka Sol, RR=Arka Sağ
        """
        k = self.lx + self.ly
        v_FL = (vx_r - vy_r - k * omega) / self.r
        v_FR = (vx_r + vy_r + k * omega) / self.r
        v_RL = (vx_r + vy_r - k * omega) / self.r
        v_RR = (vx_r - vy_r + k * omega) / self.r
        return v_FL, v_FR, v_RL, v_RR

    # ------------------------------------------------------------------

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl['vx'], ctrl['vy'], ctrl['omega']
        return float(ctrl[0]), float(ctrl[1]), float(ctrl[2])
