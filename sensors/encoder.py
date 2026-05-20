"""
sensors/encoder.py
Tekerlek enkoderi simülasyonu.

Diferansiyel sürüş için:
    Sol ve sağ tekerlek tick sayısını ölçer.
    Odometri hesabı: (v, omega) → robot hız tahmini.

Holonomic robotlar için:
    Her tekerleğin açısal hızını ölçer.

Gürültü modeli: Gaussian slip gürültüsü.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List


@dataclass
class EncoderMeasurement:
    left_ticks: int = 0
    right_ticks: int = 0
    left_speed: float = 0.0     # rad/s
    right_speed: float = 0.0    # rad/s
    left_dist: float = 0.0      # kümülatif mesafe (m) — float hassas
    right_dist: float = 0.0     # kümülatif mesafe (m) — float hassas
    timestamp: float = 0.0


class WheelEncoder:
    """
    Diferansiyel sürüş tekerlek enkoderi simülatörü.

    Args:
        ticks_per_rev:   Tur başına encoder tick sayısı.
        wheel_radius:    Tekerlek yarıçapı (m).
        wheel_base:      İki tekerlek arası mesafe (m).
        slip_noise_std:  Kayma gürültüsü oranı (0-1 arası).
    """

    def __init__(self, ticks_per_rev: int = 1000,
                 wheel_radius: float = 0.1,
                 wheel_base: float = 0.5,
                 slip_noise_std: float = 0.01):
        self.ticks_per_rev = ticks_per_rev
        self.r = wheel_radius
        self.L = wheel_base
        self.slip_noise_std = slip_noise_std

        self._left_total: int = 0
        self._right_total: int = 0
        # Float hassasiyetli mesafe takibi — tick tamsayı yuvarlama hatası yok
        self._left_dist_f: float = 0.0
        self._right_dist_f: float = 0.0
        self._time: float = 0.0

    # ------------------------------------------------------------------

    def measure(self, v_left_rad: float, v_right_rad: float,
                dt: float) -> EncoderMeasurement:
        """
        Gerçek tekerlek açısal hızlarından gürültülü enkoder ölçümü üret.

        Args:
            v_left_rad:  Sol tekerlek açısal hızı (rad/s).
            v_right_rad: Sağ tekerlek açısal hızı (rad/s).
            dt:          Zaman adımı (s).
        """
        self._time += dt

        # Kayma gürültüsü ekle
        noise_l = np.random.normal(1.0, self.slip_noise_std)
        noise_r = np.random.normal(1.0, self.slip_noise_std)

        vl_noisy = v_left_rad * noise_l
        vr_noisy = v_right_rad * noise_r

        # Float hassasiyetli mesafe takibi (odometri için kullanılacak)
        dl = vl_noisy * self.r * dt
        dr = vr_noisy * self.r * dt
        self._left_dist_f  += dl
        self._right_dist_f += dr

        # Tick hesabı (görüntüleme ve ham sayaç için)
        ticks_per_rad = self.ticks_per_rev / (2 * np.pi)
        delta_ticks_l = int(vl_noisy * dt * ticks_per_rad)
        delta_ticks_r = int(vr_noisy * dt * ticks_per_rad)

        self._left_total += delta_ticks_l
        self._right_total += delta_ticks_r

        return EncoderMeasurement(
            left_ticks=self._left_total,
            right_ticks=self._right_total,
            left_speed=vl_noisy,
            right_speed=vr_noisy,
            left_dist=self._left_dist_f,
            right_dist=self._right_dist_f,
            timestamp=self._time
        )

    def odometry(self, enc: EncoderMeasurement, prev_enc: EncoderMeasurement,
                 dt: float):
        """
        İki ardışık enkoder ölçümünden (v, omega) hız tahmini.

        Float hassasiyetli mesafe farkı kullanılır (tick tamsayı yuvarlama
        hatası yerine, ölçüm sırasında biriktirilmiş float mesafeler).

        Odometri formülleri (diferansiyel sürüş):
            dl = sol tekerlek yol farkı (m)
            dr = sağ tekerlek yol farkı (m)
            v     = (dl + dr) / (2 * dt)
            omega = (dr - dl) / (L * dt)

        Returns:
            (v_est, omega_est): Lineer (m/s) ve açısal (rad/s) hız tahmini.
        """
        dl = enc.left_dist  - prev_enc.left_dist    # float, hassas (m)
        dr = enc.right_dist - prev_enc.right_dist   # float, hassas (m)

        v_est     = (dl + dr) / (2.0 * dt) if dt > 0 else 0.0
        omega_est = (dr - dl) / (self.L * dt) if dt > 0 else 0.0
        return v_est, omega_est

    def reset(self):
        self._left_total = 0
        self._right_total = 0
        self._left_dist_f = 0.0
        self._right_dist_f = 0.0
        self._time = 0.0
