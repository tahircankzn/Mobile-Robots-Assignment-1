"""
planners/base_planner.py
Tüm yol planlayıcılar için soyut temel sınıf.

Yol Kriterleri (criteria):
    shortest  — toplam yol uzunluğunu minimize et (varsayılan)
    safest    — engel clearance'ını maximize et (güvenli mesafe)
    fastest   — dönüşleri penalize ederek geçiş süresini minimize et
    smoothest — ani yön değişikliklerini minimize et (en az dönüş)
"""

from abc import ABC, abstractmethod
from typing import List, Tuple, Optional, Callable
import numpy as np
from environment.map import GridMap
from .metrics import get_metric

# Geçerli kriter isimleri
CRITERIA = {
    "shortest":  "En kısa yol uzunluğu",
    "safest":    "En güvenli yol (engellerden uzak)",
    "fastest":   "En hızlı yol (dönüş süresi dahil)",
    "smoothest": "En düzgün yol (az dönüş)",
}


class BasePlanner(ABC):
    """
    Ortak planlayıcı arayüzü.

    Alt sınıflar plan() metodunu uygulamak zorundadır.
    """

    def __init__(self, grid_map: GridMap,
                 metric: str = "euclidean",
                 use_8_connect: bool = True,
                 criteria: str = "shortest",
                 cost_map: np.ndarray = None):
        self.map = grid_map
        self.metric_fn: Callable = get_metric(metric)
        self.metric_name = metric
        self.use_8_connect = use_8_connect
        self.criteria = criteria.lower()
        # cost_map: clearance mesafesi haritası (safest kriteri için)
        self.cost_map: np.ndarray | None = cost_map

        self.path: List[Tuple[int, int]] = []
        self._visited: set = set()
        self._nodes_expanded: int = 0

    # ------------------------------------------------------------------
    # Soyut
    # ------------------------------------------------------------------

    @abstractmethod
    def plan(self, start: Tuple[int, int],
             goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Başlangıçtan hedefe yol planla.

        Args:
            start: (row, col) başlangıç hücresi.
            goal:  (row, col) hedef hücresi.
        Returns:
            (row, col) listesi — boş liste = yol bulunamadı.
        """

    @property
    @abstractmethod
    def planner_name(self) -> str:
        """Planlayıcı adı etiketi."""

    # ------------------------------------------------------------------
    # Ortak yardımcılar
    # ------------------------------------------------------------------

    def neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        r, c = node
        candidates = (self.map.neighbors_8(r, c) if self.use_8_connect
                      else self.map.neighbors_4(r, c))
        return [n for n in candidates if self.map.is_free(*n)]

    def heuristic(self, a: Tuple[int, int],
                  b: Tuple[int, int]) -> float:
        return self.metric_fn(a, b)

    def edge_cost(self, a: Tuple[int, int], b: Tuple[int, int],
                  prev: Tuple[int, int] = None) -> float:
        """
        Kriter-bilinçli kenar maliyeti.

        Args:
            a:    Başlangıç hücresi (mevcut düğüm).
            b:    Hedef hücre (komşu).
            prev: Bir önceki hücre (yön değişikliği hesabı için).

        Kriterler:
            shortest  → basit adım mesafesi.
            safest    → mesafe * (1 + k / clearance) — engellerden uzak.
            fastest   → mesafe + fiziksel dönüş süresi.
            smoothest → mesafe + yön değişikliği ceza (lineer).
        """
        dist = self.metric_fn(a, b)

        if self.criteria == "safest" and self.cost_map is not None:
            # Komşu hücrenin ham engele yakınlığı (hücre cinsinden)
            clearance = max(float(self.cost_map[b[0], b[1]]), 0.1)
            # Clearance arttıkça ceza azalır; k=4 → 1 hücre uzaklık ~5x ek maliyet
            safety_penalty = 4.0 / (clearance + 0.5)
            dist = dist * (1.0 + safety_penalty)

        elif self.criteria in ("fastest", "smoothest") and prev is not None:
            # Yön değişikliği açısı
            dr1, dc1 = a[0] - prev[0], a[1] - prev[1]
            dr2, dc2 = b[0] - a[0],   b[1] - a[1]
            mag1 = np.hypot(dr1, dc1)
            mag2 = np.hypot(dr2, dc2)
            if mag1 > 0 and mag2 > 0:
                cos_a = (dr1 * dr2 + dc1 * dc2) / (mag1 * mag2)
                angle = np.arccos(np.clip(cos_a, -1.0, 1.0))   # 0..π rad
                if self.criteria == "fastest":
                    # Fiziksel: t_dönüş = angle / omega_max ≈ angle/π saniye
                    # v_max/omega_max ≈ 2.0/π ≈ 0.637 m/rad
                    dist += angle * 0.637
                else:  # smoothest
                    dist += angle * 0.40

        return dist

    @staticmethod
    def reconstruct_path(came_from: dict,
                         current: Tuple[int, int]) -> List[Tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))

    def stats(self) -> dict:
        return {
            "planner":        self.planner_name,
            "metric":         self.metric_name,
            "criteria":       self.criteria,
            "path_length":    len(self.path),
            "nodes_expanded": self._nodes_expanded,
        }

    def __repr__(self):
        return f"{self.planner_name}(metric={self.metric_name})"
