"""
planners/astar.py
A* (A-Star) yol planlama algoritması.

f(n) = g(n) + h(n)
    g(n): Başlangıçtan n'e gerçek maliyet.
    h(n): n'den hedefe sezgisel (heuristic) tahmini maliyet.

Seçilebilir mesafe metrikleri: euclidean, manhattan, chebyshev, octile, ...
"""

import heapq
from typing import List, Tuple, Dict
import numpy as np
from environment.map import GridMap
from .base_planner import BasePlanner


class AStar(BasePlanner):
    """
    A* algoritması.

    Args:
        grid_map:      2B grid haritası.
        metric:        Heuristic metrik ('euclidean', 'manhattan', ...).
        use_8_connect: 8-yönlü mü 4-yönlü mü hareket.
        criteria:      Yol kriteri ('shortest'|'safest'|'fastest'|'smoothest').
        cost_map:      Clearance haritası (safest kriteri için).
    """

    def __init__(self, grid_map: GridMap,
                 metric: str = "euclidean",
                 use_8_connect: bool = True,
                 criteria: str = "shortest",
                 cost_map: np.ndarray = None):
        super().__init__(grid_map, metric, use_8_connect, criteria, cost_map)

    @property
    def planner_name(self) -> str:
        return "A*"

    # ------------------------------------------------------------------

    def plan(self, start: Tuple[int, int],
             goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        A* ile en kısa yolu bul.

        Returns:
            Hücre listesi — boş = yol yok.
        """
        self._nodes_expanded = 0
        self._visited = set()

        # (f_score, g_score, node)
        open_heap: list = []
        heapq.heappush(open_heap, (0.0, 0.0, start))

        came_from: Dict[Tuple, Tuple] = {}
        g_score: Dict[Tuple, float] = {start: 0.0}
        f_score: Dict[Tuple, float] = {start: self.heuristic(start, goal)}

        while open_heap:
            _, g_cur, current = heapq.heappop(open_heap)

            if current in self._visited:
                continue
            self._visited.add(current)
            self._nodes_expanded += 1

            if current == goal:
                self.path = self.reconstruct_path(came_from, current)
                return self.path

            for neighbor in self.neighbors(current):
                parent = came_from.get(current)             # kriter için önceki
                move_cost = self.edge_cost(current, neighbor, parent)
                tentative_g = g_score.get(current, float('inf')) + move_cost

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_heap, (f, tentative_g, neighbor))

        self.path = []
        return self.path
