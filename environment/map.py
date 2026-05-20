"""
environment/map.py
2B grid tabanlı harita modülü.
- 50x50 hücrelik ortam
- 12 engel (ödev gereksinimi: en az 10)
- Başlangıç ve hedef noktaları
- Engel şişirme (inflate) ve clearance haritası
"""

import numpy as np
from collections import deque

# Hücre türleri
FREE = 0
OBSTACLE = 1
START = 2
GOAL = 3


class GridMap:
    """
    2B grid haritası.
    Her hücre FREE (0) veya OBSTACLE (1) değerini taşır.
    Koordinatlar (satır, sütun) = (y, x) formatındadır.
    """

    def __init__(self, rows: int = 50, cols: int = 50, cell_size: float = 1.0):
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size          # gerçek dünya birimi (metre)
        self.grid = np.zeros((rows, cols), dtype=np.uint8)
        self._inflated: np.ndarray | None = None  # şişirilmiş grid
        self._padding_cells: int = 0

        # Varsayılan senaryo: depo keşfi
        self.start = (2, 2)                 # (satır, sütun)
        self.goal = (47, 47)

        self._place_default_obstacles()

    # ------------------------------------------------------------------
    # Engel yerleştirme
    # ------------------------------------------------------------------

    def _place_default_obstacles(self):
        """12 dikdörtgen engel — ödev senaryosu (depo/fabrika planı)."""
        obstacles = [
            # (r_start, c_start, yükseklik, genişlik)
            (5,  10, 5, 3),
            (5,  20, 5, 3),
            (5,  30, 5, 3),
            (5,  40, 5, 3),
            (15, 5,  3, 6),
            (15, 22, 3, 6),
            (15, 38, 3, 6),
            (25, 12, 6, 3),
            (25, 28, 6, 3),
            (35, 8,  3, 8),
            (35, 30, 3, 8),
            (42, 18, 4, 4),
        ]
        for (r, c, h, w) in obstacles:
            self.grid[r:r+h, c:c+w] = OBSTACLE

    def add_obstacle(self, r: int, c: int, h: int = 1, w: int = 1):
        self.grid[r:r+h, c:c+w] = OBSTACLE

    def remove_obstacle(self, r: int, c: int, h: int = 1, w: int = 1):
        self.grid[r:r+h, c:c+w] = FREE

    # ------------------------------------------------------------------
    # Engel şişirme ve clearance
    # ------------------------------------------------------------------

    def inflate_obstacles(self, padding_cells: int = 1):
        """
        Engelleri padding_cells hücre genişliğinde dairesel olarak şişir.
        Robot yarıçapı kadar dolgu bırakarak güvenli yol planlanır.

        Args:
            padding_cells: Şişirme yarıçapı (hücre cinsinden).
        """
        self._padding_cells = padding_cells
        inflated = self.grid.copy()
        obstacle_mask = (self.grid == OBSTACLE)

        # Dairesel dilation: mesafe ≤ padding_cells olan tüm hücreler
        for dr in range(-padding_cells, padding_cells + 1):
            for dc in range(-padding_cells, padding_cells + 1):
                if dr * dr + dc * dc > padding_cells * padding_cells:
                    continue
                # obstacle_mask'ı (dr, dc) kadar kaydır
                if dr >= 0:
                    src_r = slice(0, self.rows - dr)
                    dst_r = slice(dr, self.rows)
                else:
                    src_r = slice(-dr, self.rows)
                    dst_r = slice(0, self.rows + dr)
                if dc >= 0:
                    src_c = slice(0, self.cols - dc)
                    dst_c = slice(dc, self.cols)
                else:
                    src_c = slice(-dc, self.cols)
                    dst_c = slice(0, self.cols + dc)
                inflated[dst_r, dst_c][obstacle_mask[src_r, src_c]] = OBSTACLE

        self._inflated = inflated
        # Başlangıç ve hedef her zaman açık kalsın
        self._inflated[self.start[0], self.start[1]] = FREE
        self._inflated[self.goal[0], self.goal[1]] = FREE

    def clearance_map(self) -> np.ndarray:
        """
        Her serbest hücrenin ham engele (self.grid) yakınlığını döndür.
        Dijkstra tabanlı yaklaşık Öklid mesafe dönüşümü (scipy gerektirmez).

        Returns:
            dist (rows x cols): Her hücrenin en yakın engele mesafesi (hücre).
        """
        dist = np.full((self.rows, self.cols), np.inf, dtype=float)
        q = deque()

        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r, c] == OBSTACLE:
                    dist[r, c] = 0.0
                    q.append((r, c))

        while q:
            r, c = q.popleft()
            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.rows and 0 <= nc < self.cols:
                        nd = dist[r, c] + (1.414 if dr != 0 and dc != 0 else 1.0)
                        if nd < dist[nr, nc]:
                            dist[nr, nc] = nd
                            q.append((nr, nc))
        return dist

    # ------------------------------------------------------------------
    # Sorgular
    # ------------------------------------------------------------------

    def is_free(self, r: int, c: int) -> bool:
        if not self.is_valid(r, c):
            return False
        # Şişirilmiş grid varsa onu kullan (planlayıcılar padding'e saygı gösterir)
        grid = self._inflated if self._inflated is not None else self.grid
        return int(grid[r, c]) == FREE

    def is_padding(self, r: int, c: int) -> bool:
        """Şişirme (padding) alanında ama ham engel değil → True."""
        if self._inflated is None or not self.is_valid(r, c):
            return False
        return (int(self._inflated[r, c]) == OBSTACLE and
                int(self.grid[r, c]) != OBSTACLE)

    def is_valid(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def world_to_grid(self, x: float, y: float):
        """Gerçek dünya koordinatını (x, y) → (satır, sütun) dönüştür."""
        col = int(x / self.cell_size)
        row = int(y / self.cell_size)
        return (row, col)

    def grid_to_world(self, r: int, c: int):
        """(satır, sütun) → (x, y) merkez noktası."""
        x = (c + 0.5) * self.cell_size
        y = (r + 0.5) * self.cell_size
        return (x, y)

    def neighbors_4(self, r: int, c: int):
        """4-yönlü komşular."""
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if self.is_valid(nr, nc):
                yield (nr, nc)

    def neighbors_8(self, r: int, c: int):
        """8-yönlü komşular."""
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if self.is_valid(nr, nc):
                    yield (nr, nc)

    # ------------------------------------------------------------------
    # Bilgi
    # ------------------------------------------------------------------

    def obstacle_list(self):
        """Engel hücrelerinin (r, c) listesini döndür."""
        positions = np.argwhere(self.grid == OBSTACLE)
        return [tuple(p) for p in positions]

    def __repr__(self):
        return (f"GridMap({self.rows}x{self.cols}, "
                f"start={self.start}, goal={self.goal}, "
                f"obstacles={int(np.sum(self.grid == OBSTACLE))} cells)")
