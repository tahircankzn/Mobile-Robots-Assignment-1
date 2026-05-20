"""
localization/dead_reckoning.py
Dead reckoning (ölü hesap) ile konum tahmini.

Enkoder odometrisi ve IMU gyro verisini kullanarak
robotun konumunu entegre yoluyla tahmin eder.
Birikimli hata zamanla artar (drift).
"""

import numpy as np
from typing import List, Tuple


class DeadReckoning:
    """
    Dead reckoning lokalizasyon modülü.

    State: [x, y, theta]
    Güncelleme: Euler integrasyonu ile enkoder + gyro verisi.
    """

    def __init__(self, x0: float = 0.0, y0: float = 0.0,
                 theta0: float = 0.0):
        self.x = x0
        self.y = y0
        self.theta = theta0
        self.history: List[Tuple[float, float, float]] = [(x0, y0, theta0)]

    # ------------------------------------------------------------------

    def update(self, v_encoder: float, omega_gyro: float, dt: float):
        """
        Enkoder hız tahmini ve gyro açısal hızıyla konumu güncelle.

        Args:
            v_encoder:  Lineer hız tahmini enkoder'dan (m/s).
            omega_gyro: Açısal hız ölçümü IMU'dan (rad/s).
            dt:         Zaman adımı (s).
        """
        self.x     += v_encoder * np.cos(self.theta) * dt
        self.y     += v_encoder * np.sin(self.theta) * dt
        self.theta  = self._wrap(self.theta + omega_gyro * dt)
        self.history.append((self.x, self.y, self.theta))

    def pose(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    def reset(self, x0=0.0, y0=0.0, theta0=0.0):
        self.x, self.y, self.theta = x0, y0, theta0
        self.history = [(x0, y0, theta0)]

    @staticmethod
    def _wrap(angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi
