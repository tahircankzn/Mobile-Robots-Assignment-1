"""
robots/differential_drive.py
Diferansiyel sürüş (unicycle) kinematik modeli.

Kontrol girişi: (v, omega)
    v     : lineer hız (m/s)
    omega : açısal hız (rad/s)

Kinematik denklemler:
    x'     = v * cos(θ)
    y'     = v * sin(θ)
    theta' = omega

Tekerlek hızı-kontrol dönüşümü:
    v_R = (v + omega * L/2) / r
    v_L = (v - omega * L/2) / r
"""

import numpy as np
from .base_robot import BaseRobot, RobotState


class DifferentialDrive(BaseRobot):
    """
    İki tahrikli diferansiyel sürüş robotu.
    Non-holonomic: yana doğru kayamaz.

    Args:
        wheel_base (L): İki tekerlek arası mesafe (m).
        wheel_radius (r): Tekerlek yarıçapı (m).
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0,
                 wheel_base=0.5, wheel_radius=0.1, radius=0.3):
        super().__init__(x, y, theta, radius)
        self.L = wheel_base
        self.r = wheel_radius
        self._v_max = 2.0       # m/s
        self._omega_max = np.pi # rad/s

    # ------------------------------------------------------------------

    @property
    def robot_type(self) -> str:
        return "DifferentialDrive"

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
            control_input: (v, omega) veya {'v': ..., 'omega': ...}
            dt: zaman adımı (s)

        Kinematik model:
            |omega| < eps  → düz çizgi (Euler)
            |omega| >= eps → tam dairesel yay integrasyonu (exact arc)
        """
        v, omega = self._parse_input(control_input)

        # Sınırla
        v = np.clip(v, -self._v_max, self._v_max)
        omega = np.clip(omega, -self._omega_max, self._omega_max)

        theta = self.state.theta

        if abs(omega) < 1e-6:
            # Düz çizgi integrasyonu
            self.state.x     += v * np.cos(theta) * dt
            self.state.y     += v * np.sin(theta) * dt
            self.state.theta  = self.wrap_angle(theta)
        else:
            # Tam dairesel yay integrasyonu (Euler'den daha doğru)
            # R = v / omega : anlık dönüş yarıçapı (ICR'ye mesafe)
            R = v / omega
            dtheta = omega * dt
            self.state.x     += R * (np.sin(theta + dtheta) - np.sin(theta))
            self.state.y     += R * (-np.cos(theta + dtheta) + np.cos(theta))
            self.state.theta  = self.wrap_angle(theta + dtheta)

        self.state.vx    = v * np.cos(self.state.theta)
        self.state.vy    = v * np.sin(self.state.theta)
        self.state.omega = omega

        self._record()
        return self.state

    def wheel_speeds(self, v: float, omega: float):
        """(v, omega) → (sol tekerlek hızı, sağ tekerlek hızı) rad/s.

        Diferansiyel sürüş tekerlek hızı dönüşümü:
            v_R = (v + omega * L/2) / r
            v_L = (v - omega * L/2) / r
        """
        v_R = (v + omega * self.L / 2) / self.r
        v_L = (v - omega * self.L / 2) / self.r
        return v_L, v_R

    def icr(self):
        """
        Anlık Dönüş Merkezi (ICR) hesabı.

        Diferansiyel sürüşte ICR, iki tekerlek ekseninin uzantısı
        üzerinde robot merkezinden R = v/omega uzaklıkta bulunur.

        Returns:
            (icr_x, icr_y, R) — düz gidişte None.
        """
        omega = self.state.omega
        v = np.hypot(self.state.vx, self.state.vy)
        if abs(omega) < 1e-4:
            return None
        R = v / omega   # işaretli: pozitif = sol dönüş
        theta = self.state.theta
        # ICR konumu = robot merkezinin sol dik doğrultusunda R metre
        # Sol dik: (-sin(theta), cos(theta))
        icr_x = self.state.x - R * np.sin(theta)
        icr_y = self.state.y + R * np.cos(theta)
        return (icr_x, icr_y, R)

    # ------------------------------------------------------------------

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl['v'], ctrl['omega']
        return float(ctrl[0]), float(ctrl[1])
