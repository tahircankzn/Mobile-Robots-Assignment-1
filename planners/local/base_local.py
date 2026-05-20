"""
planners/local/base_local.py
Tüm local (reaktif) yol planlayıcılar için soyut temel sınıf.

Local planlayıcılar:
    - Global harita gerektirmez
    - Her adımda anlık LiDAR verisi + hedef koordinatı → hareket komutu üretir
    - Durum makinesi (Bug0/1/2) veya saf reaktif (PotentialFields, VFH) olabilir

Ortak arayüz:
    compute(rx, ry, rtheta, ranges, beam_angles, gx, gy) -> (v, omega)
"""

from abc import ABC, abstractmethod
import numpy as np


# ─── Yardımcı fonksiyon ─────────────────────────────────────────────────────

def angle_diff(a: float, b: float) -> float:
    """[-π, π] aralığına normalize edilmiş açı farkı (a - b)."""
    d = a - b
    return (d + np.pi) % (2.0 * np.pi) - np.pi


# ─── Soyut temel sınıf ──────────────────────────────────────────────────────

class BaseLocal(ABC):
    """
    Local planlayıcı arayüzü.

    Alt sınıflar compute() metodunu uygulamak zorundadır.
    Durum makinesi gerektiren algoritmalar reset() metodunu override eder.
    """

    #: Planlayıcının görünen adı (alt sınıflar override eder)
    name: str = "BaseLocal"

    def reset(self, start_x: float, start_y: float,
              goal_x: float, goal_y: float) -> None:
        """
        Yeni navigasyon görevi başlarken çağrılır.
        Dahili durum sıfırlanır. Durumsuz algoritmalar bu metodu override etmez.
        """
        pass

    @abstractmethod
    def compute(self,
                rx: float, ry: float, rtheta: float,
                ranges: np.ndarray, beam_angles: np.ndarray,
                gx: float, gy: float) -> tuple:
        """
        Bir adımda hareket komutu hesapla.

        Args:
            rx, ry    : Robot dünya pozisyonu (m).
            rtheta    : Robot yön açısı (rad, dünya çerçevesi).
            ranges    : LiDAR filtrelenmiş mesafe okumaları (m).
            beam_angles: Her ışının robot-göreceli açısı (rad), [-π, π].
            gx, gy    : Hedef dünya koordinatı (m).

        Returns:
            (v, omega): Doğrusal hız (m/s) ve açısal hız (rad/s).
                        Değerler robotun limitlerinde kırpılacaktır.
        """
