"""
planners/rrt.py
RRT (Rapidly-exploring Random Tree) yol planlama algoritması.

Sürekli uzayda örnekleme tabanlı planlama.
Non-holonomic kısıtlara doğrudan uygulanabilir.

Referans:
    LaValle, "Rapidly-Exploring Random Trees", 1998.
"""

import numpy as np
from typing import List, Tuple, Optional
from environment.map import GridMap
from .base_planner import BasePlanner


class RRTNode:
    __slots__ = ['r', 'c', 'parent']

    def __init__(self, r: int, c: int, parent=None):
        self.r = r
        self.c = c
        self.parent = parent

    def pos(self) -> Tuple[int, int]:
        return (self.r, self.c)


class RRT(BasePlanner):
    """
    RRT algoritması (grid üzerinde).

    Args:
        max_iter:    Maksimum iterasyon sayısı.
        step_size:   Her adımda maksimum hücre hareketi.
        goal_bias:   Hedef yönünde örnekleme olasılığı [0,1].
        goal_radius: Hedefe ulaşma eşiği (hücre).
    """

    def __init__(self, grid_map: GridMap,
                 metric: str = "euclidean",
                 max_iter: int = 5000,
                 step_size: int = 3,
                 goal_bias: float = 0.15,
                 goal_radius: int = 2):
        super().__init__(grid_map, metric, use_8_connect=True)
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.goal_radius = goal_radius
        self.tree: List[RRTNode] = []

    @property
    def planner_name(self) -> str:
        return "RRT"

    # ------------------------------------------------------------------

    def plan(self, start: Tuple[int, int],
             goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        self._nodes_expanded = 0
        self.tree = [RRTNode(*start)]
        goal_node = None

        for _ in range(self.max_iter):
            # Örnekle
            q_rand = self._sample(goal)

            # En yakın düğümü bul
            q_near = self._nearest(q_rand)

            # Adım at
            q_new = self._steer(q_near, q_rand)
            if q_new is None:
                continue

            # Çarpışma kontrolü
            if not self._collision_free(q_near.pos(), q_new.pos()):
                continue

            q_new.parent = q_near
            self.tree.append(q_new)
            self._nodes_expanded += 1

            # Hedefe ulaştı mı?
            if self.heuristic(q_new.pos(), goal) <= self.goal_radius:
                goal_node = RRTNode(*goal, parent=q_new)
                self.tree.append(goal_node)
                break

        if goal_node is None:
            # En yakın düğümden yol çek
            best = min(self.tree, key=lambda n: self.heuristic(n.pos(), goal))
            if self.heuristic(best.pos(), goal) > self.goal_radius * 3:
                self.path = []
                return self.path
            goal_node = best

        self.path = self._trace_path(goal_node)
        return self.path

    # ------------------------------------------------------------------

    def _sample(self, goal: Tuple[int, int]) -> Tuple[int, int]:
        if np.random.random() < self.goal_bias:
            return goal
        r = np.random.randint(0, self.map.rows)
        c = np.random.randint(0, self.map.cols)
        return (r, c)

    def _nearest(self, q_rand: Tuple[int, int]) -> RRTNode:
        return min(self.tree, key=lambda n: self.heuristic(n.pos(), q_rand))

    def _steer(self, q_near: RRTNode,
               q_rand: Tuple[int, int]) -> Optional[RRTNode]:
        dr = q_rand[0] - q_near.r
        dc = q_rand[1] - q_near.c
        dist = np.hypot(dr, dc)
        if dist == 0:
            return None
        scale = min(self.step_size / dist, 1.0)
        nr = int(round(q_near.r + dr * scale))
        nc = int(round(q_near.c + dc * scale))
        if not self.map.is_valid(nr, nc) or not self.map.is_free(nr, nc):
            return None
        return RRTNode(nr, nc)

    def _collision_free(self, a: Tuple[int, int],
                        b: Tuple[int, int]) -> bool:
        """Bresenham doğru kontrolü."""
        r0, c0 = a
        r1, c1 = b
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        while True:
            if not self.map.is_free(r0, c0):
                return False
            if r0 == r1 and c0 == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r0 += sr
            if e2 < dr:
                err += dr
                c0 += sc
        return True

    def _trace_path(self, node: RRTNode) -> List[Tuple[int, int]]:
        path = []
        while node is not None:
            path.append(node.pos())
            node = node.parent
        return list(reversed(path))
