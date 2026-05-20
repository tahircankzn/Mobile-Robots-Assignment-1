"""
robots/__init__.py
Robot kütüphanesi — tüm robot tiplerini dışa aktar.
Kullanım:
    from robots import DifferentialDrive, OmniWheel, Ackermann, Mecanum
    from robots import get_robot, ROBOT_TYPES
"""

from .differential_drive import DifferentialDrive
from .omniwheel import OmniWheel
from .ackermann import Ackermann
from .mecanum import Mecanum
from .drone import Quadrotor
from .vtol import VTOL
from .fixed_wing import FixedWing
from .quadruped import Quadruped
from .hexapod import Hexapod
from .snake_robot import SnakeRobot
from .bipedal import Bipedal
from .base_robot import BaseRobot, RobotState

# Seçim menüsü için isim → sınıf eşlemesi
ROBOT_TYPES: dict = {
    "differential": DifferentialDrive,
    "omni":         OmniWheel,
    "ackermann":    Ackermann,
    "mecanum":      Mecanum,
    "drone":        Quadrotor,
    "vtol":         VTOL,
    "fixedwing":    FixedWing,
    "quadruped":    Quadruped,
    "hexapod":      Hexapod,
    "snake":        SnakeRobot,
    "bipedal":      Bipedal,
}


def get_robot(robot_type: str, x=0.0, y=0.0, theta=0.0, **kwargs) -> BaseRobot:
    """
    İsme göre robot örneği oluştur.

    Args:
        robot_type: 'differential' | 'omni' | 'ackermann' | 'mecanum'
        x, y, theta: başlangıç pozu
        **kwargs: robot sınıfına özgü parametreler
    Returns:
        BaseRobot alt sınıf örneği
    Raises:
        ValueError: geçersiz robot tipi
    """
    key = robot_type.lower().strip()
    if key not in ROBOT_TYPES:
        raise ValueError(
            f"Bilinmeyen robot tipi: '{robot_type}'. "
            f"Geçerli seçenekler: {list(ROBOT_TYPES.keys())}"
        )
    return ROBOT_TYPES[key](x=x, y=y, theta=theta, **kwargs)


__all__ = [
    "BaseRobot", "RobotState",
    "DifferentialDrive", "OmniWheel", "Ackermann", "Mecanum",
    "ROBOT_TYPES", "get_robot",
]
