"""
robots/base_robot.py
Tüm robot türleri için soyut temel sınıf.
Her alt sınıf kinematic_step() metodunu uygulamak zorundadır.
"""

from abc import ABC, abstractmethod
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class RobotState:
    """Robotun anlık durumu."""
    x: float = 0.0          # dünya koordinatı (metre)
    y: float = 0.0
    theta: float = 0.0      # yön açısı (radyan)
    vx: float = 0.0         # dünya x hızı (m/s)
    vy: float = 0.0         # dünya y hızı (m/s)
    omega: float = 0.0      # açısal hız (rad/s)


class BaseRobot(ABC):
    """
    Tüm robot modelleri için ortak arayüz.

    Alt sınıflar şunları uygulamalıdır:
        kinematic_step(control_input, dt) -> RobotState
        max_linear_speed (property)
        max_angular_speed (property)
        robot_type (property) -> str
    """

    def __init__(self, x: float = 0.0, y: float = 0.0,
                 theta: float = 0.0, radius: float = 0.3):
        self.state = RobotState(x=x, y=y, theta=theta)
        self.radius = radius            # gövde yarıçapı (çarpışma tespiti için)
        self.history: List[Tuple[float, float, float]] = []  # (x, y, theta)
        self._record()

    # ------------------------------------------------------------------
    # Soyut metodlar
    # ------------------------------------------------------------------

    @abstractmethod
    def kinematic_step(self, control_input, dt: float) -> RobotState:
        """
        Kontrol girişini uygula, durumu güncelle, yeni durumu döndür.

        Args:
            control_input: Alt sınıfa özgü kontrol (tuple/dict).
            dt:            Zaman adımı (saniye).
        Returns:
            Güncellenmiş RobotState.
        """

    @property
    @abstractmethod
    def max_linear_speed(self) -> float:
        """Maksimum lineer hız (m/s)."""

    @property
    @abstractmethod
    def max_angular_speed(self) -> float:
        """Maksimum açısal hız (rad/s)."""

    @property
    @abstractmethod
    def robot_type(self) -> str:
        """Robot tipi etiketi."""

    # ------------------------------------------------------------------
    # Ortak metodlar
    # ------------------------------------------------------------------

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.state = RobotState(x=x, y=y, theta=theta)
        self.history.clear()
        self._record()

    def _record(self):
        self.history.append((self.state.x, self.state.y, self.state.theta))

    def pose(self) -> Tuple[float, float, float]:
        """(x, y, theta) döndür."""
        return (self.state.x, self.state.y, self.state.theta)

    def position(self) -> Tuple[float, float]:
        return (self.state.x, self.state.y)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        """Açıyı [-π, π] aralığına sar."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def __repr__(self):
        s = self.state
        return (f"{self.robot_type}(x={s.x:.2f}, y={s.y:.2f}, "
                f"θ={np.degrees(s.theta):.1f}°)")
