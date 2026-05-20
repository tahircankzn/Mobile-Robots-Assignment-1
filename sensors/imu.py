"""
sensors/imu.py
IMU (Inertial Measurement Unit) sensör simülasyonu.

Ölçümler:
- Açısal hız (gyroscope): omega + bias + gürültü
- Lineer ivme (accelerometer): a_x, a_y + bias + gürültü

Gürültü modeli:
- Gaussian beyaz gürültü (AWGN)
- Sabit bias terimi (gerçek IMU davranışı)
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class IMUMeasurement:
    omega_measured: float     # gyro ölçümü (rad/s)
    ax_measured: float        # ivmeölçer x (m/s²)
    ay_measured: float        # ivmeölçer y (m/s²)
    timestamp: float = 0.0    # simülasyon zamanı (s)


class IMU:
    """
    IMU sensörü simülatörü.

    Args:
        gyro_noise_std:  Gyro gürültüsü std (rad/s).
        gyro_bias:       Gyro sabit bias (rad/s).
        accel_noise_std: İvmeölçer gürültüsü std (m/s²).
        accel_bias:      İvmeölçer sabit bias (m/s²).
    """

    def __init__(self, gyro_noise_std: float = 0.01,
                 gyro_bias: float = 0.005,
                 accel_noise_std: float = 0.05,
                 accel_bias: float = 0.02):
        self.gyro_noise_std = gyro_noise_std
        self.gyro_bias = gyro_bias
        self.accel_noise_std = accel_noise_std
        self.accel_bias = accel_bias
        self._time = 0.0

    # ------------------------------------------------------------------

    def measure(self, true_omega: float, true_ax: float,
                true_ay: float, dt: float) -> IMUMeasurement:
        """
        Gerçek kinematik değerlerden gürültülü IMU ölçümü üret.

        Args:
            true_omega: Gerçek açısal hız (rad/s).
            true_ax:    Gerçek x ivmesi (m/s²).
            true_ay:    Gerçek y ivmesi (m/s²).
            dt:         Zaman adımı (s).

        Returns:
            IMUMeasurement
        """
        self._time += dt

        omega_m = (true_omega
                   + self.gyro_bias
                   + np.random.normal(0, self.gyro_noise_std))

        ax_m = (true_ax
                + self.accel_bias
                + np.random.normal(0, self.accel_noise_std))

        ay_m = (true_ay
                + self.accel_bias
                + np.random.normal(0, self.accel_noise_std))

        return IMUMeasurement(
            omega_measured=omega_m,
            ax_measured=ax_m,
            ay_measured=ay_m,
            timestamp=self._time
        )

    def reset(self):
        self._time = 0.0
