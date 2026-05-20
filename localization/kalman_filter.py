"""
localization/kalman_filter.py
Genişletilmiş Kalman Filtresi (EKF) tabanlı lokalizasyon.

State vektörü: [x, y, theta]

Süreç modeli (non-holonomic unicycle):
    x_{k+1}     = x_k + v*cos(theta_k)*dt
    y_{k+1}     = y_k + v*sin(theta_k)*dt
    theta_{k+1} = theta_k + omega*dt

Ölçüm modeli:
    z = H * x + noise
    Burada H = I (konum + açı doğrudan ölçülebilir varsayımı)
    LiDAR, IMU ve enkoder verileri füzyon için kullanılır.

Referans:
    [1] Thrun et al., "Probabilistic Robotics", MIT Press, 2005.
"""

import numpy as np
from typing import Tuple, List


class EKF:
    """
    Genişletilmiş Kalman Filtresi (EKF) lokalizasyon.

    State: x = [x, y, theta]^T  (3x1)
    Covariance: P (3x3)

    Args:
        x0, y0, theta0: Başlangıç konumu.
        Q: Süreç gürültüsü kovaryans matrisi (3x3).
        R: Ölçüm gürültüsü kovaryans matrisi (3x3).
    """

    def __init__(self, x0: float = 0.0, y0: float = 0.0,
                 theta0: float = 0.0,
                 Q: np.ndarray = None,
                 R: np.ndarray = None):

        # Durum vektörü
        self.x = np.array([x0, y0, theta0], dtype=float)

        # Kovaryans matrisi
        self.P = np.diag([0.1, 0.1, 0.05])

        # Süreç gürültüsü
        self.Q = Q if Q is not None else np.diag([0.02, 0.02, 0.01])

        # Ölçüm gürültüsü (LiDAR+IMU+Enkoder füzyonu)
        self.R = R if R is not None else np.diag([0.1, 0.1, 0.05])

        # Ölçüm matrisi (doğrusal)
        self.H = np.eye(3)

        # Geçmiş
        self.history: List[Tuple[float, float, float]] = [tuple(self.x)]

    # ------------------------------------------------------------------
    # EKF Tahmin Adımı (Predict)
    # ------------------------------------------------------------------

    def predict(self, v: float, omega: float, dt: float):
        """
        Kontrol girişine göre durum tahmini yap.

        Args:
            v:     Lineer hız (m/s).
            omega: Açısal hız (rad/s).
            dt:    Zaman adımı (s).
        """
        x, y, theta = self.x

        # Süreç modeli (non-linear)
        x_pred = x + v * np.cos(theta) * dt
        y_pred = y + v * np.sin(theta) * dt
        t_pred = self._wrap(theta + omega * dt)
        self.x = np.array([x_pred, y_pred, t_pred])

        # Jakobiyan F (süreç modelinin doğrusal yaklaşımı)
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0,  1]
        ])

        # Kovaryans güncelleme
        self.P = F @ self.P @ F.T + self.Q

    # ------------------------------------------------------------------
    # EKF Güncelleme Adımı (Update)
    # ------------------------------------------------------------------

    def update(self, z: np.ndarray):
        """
        Sensör ölçümüyle tahmin düzelt.

        Args:
            z: Ölçüm vektörü [x_meas, y_meas, theta_meas].
        """
        z = np.asarray(z, dtype=float)
        z[2] = self._wrap(z[2])

        # Yenilik (innovation)
        y_inn = z - self.H @ self.x
        y_inn[2] = self._wrap(y_inn[2])

        # Kalman kazancı
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Durum güncelleme
        self.x = self.x + K @ y_inn
        self.x[2] = self._wrap(self.x[2])

        # Kovaryans güncelleme (Joseph form — nümerik kararlılık)
        I_KH = np.eye(3) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        self.history.append(tuple(self.x))

    # ------------------------------------------------------------------
    # Sensör Füzyon Yardımcıları
    # ------------------------------------------------------------------

    def fuse(self, v_enc: float, omega_imu: float, dt: float,
             lidar_xy: Tuple[float, float] = None):
        """
        Tek adımda predict + update:
        - Enkoder'dan v, IMU'dan omega ile tahmin.
        - LiDAR varsa x,y güncelleme.

        Args:
            v_enc:    Enkoder lineer hız tahmini.
            omega_imu: IMU gyro açısal hız ölçümü.
            dt:        Zaman adımı.
            lidar_xy:  (x, y) LiDAR konumu tahmini (opsiyonel).
        """
        self.predict(v_enc, omega_imu, dt)

        # Ölçüm: varsa lidar_xy, yoksa sadece tahmin açısı kullan
        if lidar_xy is not None:
            z = np.array([lidar_xy[0], lidar_xy[1], self.x[2]])
            self.update(z)
        else:
            # Sadece açı güncelleme (IMU yeterince güvenilir varsayımı)
            z = np.array([self.x[0], self.x[1], omega_imu * dt + self.x[2]])
            self.update(z)

    # ------------------------------------------------------------------

    def pose(self) -> Tuple[float, float, float]:
        return (self.x[0], self.x[1], self.x[2])

    def reset(self, x0=0.0, y0=0.0, theta0=0.0):
        self.x = np.array([x0, y0, theta0], dtype=float)
        self.P = np.diag([0.1, 0.1, 0.05])
        self.history = [tuple(self.x)]

    @staticmethod
    def _wrap(angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi
