"""
planners/dstar.py
D* Lite yol planlama algoritması.

D* Lite, dinamik ortamlarda yeniden planlama için kullanılır.
Hedeften başlangıca doğru arama yapar (geriye doğru A*).
Engeller değiştiğinde yalnızca etkilenen bölgeyi yeniden hesaplar.

Referans:
    Koenig & Likhachev, "D* Lite", AAAI 2002.
"""

import heapq
from typing import List, Tuple, Dict
from environment.map import GridMap
from .base_planner import BasePlanner

INF = float('inf')


class DStarLite(BasePlanner):
    """
    D* Lite algoritması.
    Dinamik engel güncellemelerinde etkin yeniden planlama.
    """

    def __init__(self, grid_map: GridMap,
                 metric: str = "euclidean",
                 use_8_connect: bool = True):
        super().__init__(grid_map, metric, use_8_connect)
        self._start: Tuple[int, int] = (0, 0)
        self._goal: Tuple[int, int] = (0, 0)
        self._km: float = 0.0
        self._g: Dict[Tuple, float] = {}
        self._rhs: Dict[Tuple, float] = {}
        self._open: list = []  # min-heap

    @property
    def planner_name(self) -> str:
        return "D* Lite"

    # ------------------------------------------------------------------
    # D* Lite çekirdek
    # ------------------------------------------------------------------

    def _h(self, a, b) -> float:
        return self.heuristic(a, b)

    def _g(self, node) -> float:
        return self._g_map.get(node, INF)

    def _rhs(self, node) -> float:
        return self._rhs_map.get(node, INF)

    def _key(self, node) -> Tuple[float, float]:
        g = self._g_map.get(node, INF)
        r = self._rhs_map.get(node, INF)
        mn = min(g, r)
        return (mn + self._h(self._start, node) + self._km, mn)

    def _initialize(self, start, goal):
        self._start = start
        self._goal = goal
        self._km = 0.0
        self._g_map: Dict = {}
        self._rhs_map: Dict = {goal: 0.0}
        self._open_set: list = []
        self._in_open: Dict = {}
        self._push(goal)

    def _push(self, node):
        k = self._key(node)
        heapq.heappush(self._open_set, (k, node))
        self._in_open[node] = k

    def _update_vertex(self, u):
        if u != self._goal:
            best = INF
            for nb in self.neighbors(u):
                c = self.heuristic(u, nb)
                best = min(best, c + self._g_map.get(nb, INF))
            self._rhs_map[u] = best

        self._in_open.pop(u, None)
        if self._g_map.get(u, INF) != self._rhs_map.get(u, INF):
            self._push(u)

    def _compute_shortest_path(self):
        self._nodes_expanded = 0
        while self._open_set:
            k_old, u = heapq.heappop(self._open_set)
            if self._in_open.get(u) != k_old:
                continue  # eski giriş (bayat)

            # Hemen işaretlemeyi kaldır — aynı anahtarlı yinelenen girişleri bayat yapar
            self._in_open.pop(u, None)

            k_new = self._key(u)
            if k_old < k_new:
                self._push(u)  # güncel anahtarla yeniden ekle
                continue

            # Sonlandırma: en üst anahtar ≥ başlangıç anahtarı VE başlangıç tutarlı
            s_k = self._key(self._start)
            if (k_old >= s_k and
                    self._g_map.get(self._start, INF) ==
                    self._rhs_map.get(self._start, INF)):
                break

            g_u = self._g_map.get(u, INF)
            rhs_u = self._rhs_map.get(u, INF)
            self._nodes_expanded += 1

            if g_u > rhs_u:
                self._g_map[u] = rhs_u
                for nb in self.neighbors(u):
                    self._update_vertex(nb)
            else:
                self._g_map[u] = INF
                self._update_vertex(u)
                for nb in self.neighbors(u):
                    self._update_vertex(nb)

    def _extract_path(self) -> List[Tuple[int, int]]:
        if self._g_map.get(self._start, INF) == INF:
            return []
        path = [self._start]
        current = self._start
        for _ in range(self.map.rows * self.map.cols):
            nbs = self.neighbors(current)
            if not nbs:
                break
            current = min(nbs, key=lambda n: (
                self.heuristic(current, n) + self._g_map.get(n, INF)
            ))
            path.append(current)
            if current == self._goal:
                break
        return path

    # ------------------------------------------------------------------

    def plan(self, start: Tuple[int, int],
             goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        self._initialize(start, goal)
        self._compute_shortest_path()
        self.path = self._extract_path()
        return self.path

    def replan(self, new_start: Tuple[int, int],
               changed_cells: List[Tuple[int, int]] = None):
        """
        Engel değişikliği sonrası yeniden planla.

        Args:
            new_start:     Robotun güncel konumu.
            changed_cells: Değişen hücre listesi (engel eklendi/kaldırıldı).
        """
        self._km += self._h(self._start, new_start)
        self._start = new_start

        if changed_cells:
            for cell in changed_cells:
                self._update_vertex(cell)
                for nb in self.neighbors(cell):
                    self._update_vertex(nb)

        self._compute_shortest_path()
        self.path = self._extract_path()
        return self.path
