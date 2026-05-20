"""
planners/rrt_star.py
RRT* (RRT-Star) yol planlama algoritması.

RRT'nin optimal versiyonu:
- Yeniden bağlantı (rewire) adımı ile asimptotik optimal yol bulur.
- Yeterli iterasyon ile en kısa yola yakınsar.

Referans:
    Karaman & Frazzoli, "Sampling-based algorithms for optimal motion planning",
    IJRR 2011.
"""

import numpy as np
from typing import List, Tuple, Optional
from environment.map import GridMap
from .rrt import RRT, RRTNode


class RRTStar(RRT):
    """
    RRT* algoritması.

    Ek parametre:
        rewire_radius: Yeniden bağlantı için komşu arama yarıçapı (hücre).
    """

    def __init__(self, grid_map: GridMap,
                 metric: str = "euclidean",
                 max_iter: int = 5000,
                 step_size: int = 3,
                 goal_bias: float = 0.15,
                 goal_radius: int = 2,
                 rewire_radius: int = 6):
        super().__init__(grid_map, metric, max_iter, step_size,
                         goal_bias, goal_radius)
        self.rewire_radius = rewire_radius
        self._cost: dict = {}  # node → başlangıçtan maliyet

    @property
    def planner_name(self) -> str:
        return "RRT*"

    # ------------------------------------------------------------------

    def plan(self, start: Tuple[int, int],
             goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        self._nodes_expanded = 0
        self.tree = [RRTNode(*start)]
        self._cost = {id(self.tree[0]): 0.0}
        goal_node: Optional[RRTNode] = None
        best_cost = float('inf')

        for _ in range(self.max_iter):
            q_rand = self._sample(goal)
            q_near = self._nearest(q_rand)
            q_new = self._steer(q_near, q_rand)

            if q_new is None:
                continue
            if not self._collision_free(q_near.pos(), q_new.pos()):
                continue

            # Near nodes bul
            near_nodes = self._near_nodes(q_new)

            # En düşük maliyetli parent seç
            q_min = q_near
            c_min = (self._cost.get(id(q_near), float('inf'))
                     + self.heuristic(q_near.pos(), q_new.pos()))

            for q_n in near_nodes:
                if not self._collision_free(q_n.pos(), q_new.pos()):
                    continue
                c = (self._cost.get(id(q_n), float('inf'))
                     + self.heuristic(q_n.pos(), q_new.pos()))
                if c < c_min:
                    c_min = c
                    q_min = q_n

            q_new.parent = q_min
            self._cost[id(q_new)] = c_min
            self.tree.append(q_new)
            self._nodes_expanded += 1

            # Rewire
            for q_n in near_nodes:
                c_through_new = (c_min
                                 + self.heuristic(q_new.pos(), q_n.pos()))
                if (c_through_new < self._cost.get(id(q_n), float('inf'))
                        and self._collision_free(q_new.pos(), q_n.pos())):
                    q_n.parent = q_new
                    self._cost[id(q_n)] = c_through_new

            # Hedef kontrolü
            if self.heuristic(q_new.pos(), goal) <= self.goal_radius:
                total = c_min + self.heuristic(q_new.pos(), goal)
                if total < best_cost:
                    best_cost = total
                    goal_node = q_new

        if goal_node is None:
            best = min(self.tree,
                       key=lambda n: self.heuristic(n.pos(), goal))
            if self.heuristic(best.pos(), goal) > self.goal_radius * 3:
                self.path = []
                return self.path
            goal_node = best

        self.path = self._trace_path(goal_node)
        return self.path

    # ------------------------------------------------------------------

    def _near_nodes(self, q_new: RRTNode) -> List[RRTNode]:
        return [n for n in self.tree
                if self.heuristic(n.pos(), q_new.pos()) <= self.rewire_radius]
