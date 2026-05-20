"""
robots/drone.py
Kuadrotor (Drone) kinematik modeli.

Basitleştirilmiş 2D kuadrotor: XY düzleminde holonomik hareket.
Gerçek sistemde attitude control loop, motor mixing, thrust allocation
vb. vardır; burada velocity-level kinematik model kullanılmıştır.

Kontrol girişi: (vx, vy, omega)
    vx, vy : dünya koordinatlarında lineer hız bileşenleri (m/s)
    omega  : açısal hız / yaw rate (rad/s)

Holonomic: herhangi bir yönde, bağımsız olarak hareket edebilir.
4 rotor X-konfigürasyonunda (45°, 135°, 225°, 315°).
"""

import numpy as np
from .base_robot import BaseRobot, RobotState


class Quadrotor(BaseRobot):
    """
    Kuadrotor / drone kinematik modeli (2D, holonomik).

    Özellikler:
        arm_len   : motor kollarının uzunluğu (m)
        v_max     : maksimum lineer hız (m/s)
        omega_max : maksimum yaw hızı (rad/s)
    """

    def __init__(self, x: float = 0.0, y: float = 0.0,
                 theta: float = 0.0, arm_len: float = 0.25,
                 radius: float = 0.35):
        super().__init__(x, y, theta, radius)
        self.arm_len    = arm_len
        self._v_max     = 4.0      # m/s
        self._omega_max = 3.0      # rad/s

    # ── Arayüz ──────────────────────────────────────────────────────

    @property
    def robot_type(self) -> str:
        return "Quadrotor"

    @property
    def max_linear_speed(self) -> float:
        return self._v_max

    @property
    def max_angular_speed(self) -> float:
        return self._omega_max

    # ── Kinematik ────────────────────────────────────────────────────

    def kinematic_step(self, control_input, dt: float) -> RobotState:
        """
        Holonomik adım: dünya koordinatlarında hız vektörü + yaw rate.

        Args:
            control_input: (vx, vy, omega)
            dt: zaman adımı (s)
        """
        vx, vy, omega = self._parse_input(control_input)

        # Hız sınırlaması
        speed = np.hypot(vx, vy)
        if speed > self._v_max:
            scale = self._v_max / speed
            vx, vy = vx * scale, vy * scale
        omega = float(np.clip(omega, -self._omega_max, self._omega_max))

        self.state.x     += vx * dt
        self.state.y     += vy * dt
        self.state.theta  = self.wrap_angle(self.state.theta + omega * dt)
        self.state.vx     = vx
        self.state.vy     = vy
        self.state.omega  = omega

        self._record()
        return self.state

    # ── Yardımcı ────────────────────────────────────────────────────

    @staticmethod
    def _parse_input(ctrl):
        if isinstance(ctrl, dict):
            return ctrl['vx'], ctrl['vy'], ctrl['omega']
        c = list(ctrl)
        if len(c) == 2:          # (v, omega) → holonomik dönüşüm
            v, omega = float(c[0]), float(c[1])
            return v, 0.0, omega
        return float(c[0]), float(c[1]), float(c[2])
