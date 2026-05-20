"""
robots/omniwheel.py
Üç tekerlekli omni-yönlü (holonomic) kinematik modeli.

Kontrol girişi: (vx, vy, omega)
    vx    : robot çerçevesinde x hızı (m/s) — ileri/geri
    vy    : robot çerçevesinde y hızı (m/s) — sol/sağ kayma
    omega : açısal hız (rad/s)

Dünya çerçevesine dönüşüm:
    X' = vx*cos(θ) - vy*sin(θ)
    Y' = vx*sin(θ) + vy*cos(θ)
    θ' = omega

Tekerlek hızları (3 tekerlek, 120° aralıklı):
    φ_k = k * 2π/3  (k = 0, 1, 2)
    v_k = -sin(φ_k)*vx + cos(φ_k)*vy + R*omega
"""

import numpy as np
from .base_robot import BaseRobot, RobotState


class OmniWheel(BaseRobot):
    """
    3 tekerlekli omni-yönlü robot.
    Holonomic: her yöne serbestçe hareket edebilir.

    Args:
        wheel_radius (r): Tekerlek yarıçapı (m).
        base_radius (R):  Merkez-tekerlek arası mesafe (m).
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0,
                 wheel_radius=0.05, base_radius=0.2, radius=0.25):
        super().__init__(x, y, theta, radius)
        self.r = wheel_radius
        self.R = base_radius
        self._v_max = 2.0
        self._omega_max = 2 * np.pi

    # ------------------------------------------------------------------

    @property
    def robot_type(self) -> str:
        return "OmniWheel"

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

        # Hız sınırlama
        v_mag = np.hypot(vx_r, vy_r)
        if v_mag > self._v_max:
            scale = self._v_max / v_mag
            vx_r *= scale
            vy_r *= scale
        omega = np.clip(omega, -self._omega_max, self._omega_max)

        theta = self.state.theta

        # Robot çerçevesinden dünya çerçevesine dönüşüm
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
        """(vx_robot, vy_robot, omega) → 3 tekerlek açısal hızları (rad/s)."""
        speeds = []
        for k in range(3):
            phi = k * 2 * np.pi / 3
            v_k = (-np.sin(phi) * vx_r + np.cos(phi) * vy_r + self.R * omega)
            speeds.append(v_k / self.r)
        return speeds

    # ------------------------------------------------------------------

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl['vx'], ctrl['vy'], ctrl['omega']
        return float(ctrl[0]), float(ctrl[1]), float(ctrl[2])
