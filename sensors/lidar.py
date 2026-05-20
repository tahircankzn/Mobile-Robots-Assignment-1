"""
sensors/lidar.py
2B LiDAR sensör simülasyonu.

Özellikler:
- Çevresel tarama (360° veya özel açı aralığı)
- Gaussian gürültü modeli
- Mesafe eşikleme (min/max range)
- Basit engel kümeleme
- Ham ve filtrelenmiş veri çıktısı
"""

import numpy as np
from typing import List, Tuple, Optional
from environment.map import GridMap


class LiDAR:
    """
    2B LiDAR sensör simülatörü.

    Args:
        num_beams:   Tarama ışını sayısı.
        max_range:   Maksimum algılama mesafesi (metre).
        min_range:   Minimum algılama mesafesi (metre).
        fov:         Tarama açısı (radyan, varsayılan 2π = 360°).
        noise_std:   Ölçüm gürültüsü standart sapması (metre).
        resolution:  Işın adım büyüklüğü (metre).
    """

    def __init__(self, num_beams: int = 360, max_range: float = 10.0,
                 min_range: float = 0.1, fov: float = 2 * np.pi,
                 noise_std: float = 0.05, resolution: float = 0.05):
        self.num_beams = num_beams
        self.max_range = max_range
        self.min_range = min_range
        self.fov = fov
        self.noise_std = noise_std
        self.resolution = resolution

        # Işın açıları
        self.beam_angles = np.linspace(-fov / 2, fov / 2, num_beams)

        # Son tarama sonuçları
        self.raw_ranges: np.ndarray = np.full(num_beams, max_range)
        self.filtered_ranges: np.ndarray = np.full(num_beams, max_range)

    # ------------------------------------------------------------------
    # Ana tarama
    # ------------------------------------------------------------------

    def scan(self, robot_x: float, robot_y: float, robot_theta: float,
             grid_map: GridMap) -> Tuple[np.ndarray, np.ndarray]:
        """
        Robotun güncel pozisyonundan LiDAR taraması gerçekleştir.

        Args:
            robot_x, robot_y: Dünya koordinatı (metre).
            robot_theta:       Yön açısı (radyan).
            grid_map:          2B engel haritası.

        Returns:
            (raw_ranges, filtered_ranges): Her ışın için mesafe değerleri.
        """
        raw = np.zeros(self.num_beams)

        for i, angle_offset in enumerate(self.beam_angles):
            angle = robot_theta + angle_offset
            dist = self._cast_ray(robot_x, robot_y, angle, grid_map)

            # Gürültü ekle
            noisy = dist + np.random.normal(0, self.noise_std)
            noisy = np.clip(noisy, self.min_range, self.max_range)
            raw[i] = noisy

        self.raw_ranges = raw.copy()
        self.filtered_ranges = self._median_filter(raw, window=5)

        return self.raw_ranges, self.filtered_ranges

    # ------------------------------------------------------------------
    # Işın döküm (ray casting)
    # ------------------------------------------------------------------

    def _cast_ray(self, ox: float, oy: float, angle: float,
                  grid_map: GridMap) -> float:
        """Verilen açıda engele kadar mesafeyi hesapla."""
        dx = np.cos(angle) * self.resolution
        dy = np.sin(angle) * self.resolution

        x, y = ox, oy
        dist = 0.0

        while dist < self.max_range:
            x += dx
            y += dy
            dist += self.resolution

            r, c = grid_map.world_to_grid(x, y)
            if not grid_map.is_valid(r, c):
                return dist
            if not grid_map.is_free(r, c):
                return dist

        return self.max_range

    # ------------------------------------------------------------------
    # Filtreleme
    # ------------------------------------------------------------------

    @staticmethod
    def _median_filter(ranges: np.ndarray, window: int = 5) -> np.ndarray:
        """Dairesel medyan filtresi."""
        half = window // 2
        n = len(ranges)
        filtered = np.zeros(n)
        for i in range(n):
            idx = [(i + j - half) % n for j in range(window)]
            filtered[i] = np.median(ranges[idx])
        return filtered

    # ------------------------------------------------------------------
    # Nokta bulutu ve kümeleme
    # ------------------------------------------------------------------

    def to_point_cloud(self, robot_x: float, robot_y: float,
                       robot_theta: float,
                       use_filtered: bool = True) -> np.ndarray:
        """
        LiDAR mesafelerini dünya koordinatlarında (x, y) nokta bulutuna çevir.

        Returns:
            Nx2 numpy dizisi [[x0,y0], [x1,y1], ...]
        """
        ranges = self.filtered_ranges if use_filtered else self.raw_ranges
        points = []
        for i, angle_offset in enumerate(self.beam_angles):
            r = ranges[i]
            if r < self.max_range:
                angle = robot_theta + angle_offset
                px = robot_x + r * np.cos(angle)
                py = robot_y + r * np.sin(angle)
                points.append([px, py])
        return np.array(points) if points else np.empty((0, 2))

    def detect_obstacles(self, robot_x: float, robot_y: float,
                         robot_theta: float,
                         threshold: float = 0.5) -> List[Tuple[float, float]]:
        """
        Basit kümeleme: yakın nokta gruplarından engel merkezi tahmini.

        Args:
            threshold: Aynı küme sayılacak max nokta arası mesafe (m).
        Returns:
            Engel merkezi koordinatları listesi.
        """
        cloud = self.to_point_cloud(robot_x, robot_y, robot_theta)
        if len(cloud) == 0:
            return []

        clusters: List[List] = []
        current_cluster: List = [cloud[0]]

        for pt in cloud[1:]:
            if np.linalg.norm(pt - current_cluster[-1]) < threshold:
                current_cluster.append(pt)
            else:
                clusters.append(current_cluster)
                current_cluster = [pt]
        clusters.append(current_cluster)

        centers = []
        for cl in clusters:
            arr = np.array(cl)
            centers.append(tuple(arr.mean(axis=0)))
        return centers
