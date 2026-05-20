"""
planners/dijkstra.py
Dijkstra yol planlama algoritması.

A*'ın özel hali: h(n) = 0 (heuristic yok).
Garantili en kısa yol, ancak A*'dan daha yavaş.
"""

import heapq
from typing import List, Tuple, Dict
import numpy as np
from environment.map import GridMap
from .base_planner import BasePlanner


class Dijkstra(BasePlanner):

    def __init__(self, grid_map: GridMap,
                 metric: str = "euclidean",
                 use_8_connect: bool = True,
                 criteria: str = "shortest",
                 cost_map: np.ndarray = None):
        super().__init__(grid_map, metric, use_8_connect, criteria, cost_map)

    @property
    def planner_name(self) -> str:
        return "Dijkstra"

    # ------------------------------------------------------------------

    def plan(self, start: Tuple[int, int],
             goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        self._nodes_expanded = 0
        self._visited = set()

        open_heap: list = []
        heapq.heappush(open_heap, (0.0, start))

        came_from: Dict[Tuple, Tuple] = {}
        dist: Dict[Tuple, float] = {start: 0.0}

        while open_heap:
            d_cur, current = heapq.heappop(open_heap)

            if current in self._visited:
                continue
            self._visited.add(current)
            self._nodes_expanded += 1

            if current == goal:
                self.path = self.reconstruct_path(came_from, current)
                return self.path

            for neighbor in self.neighbors(current):
                parent = came_from.get(current)
                move_cost = self.edge_cost(current, neighbor, parent)
                new_dist = dist.get(current, float('inf')) + move_cost

                if new_dist < dist.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    dist[neighbor] = new_dist
                    heapq.heappush(open_heap, (new_dist, neighbor))

        self.path = []
        return self.path
