"""
planners/__init__.py
Yol planlama kütüphanesi — global ve local tüm planlayıcıları dışa aktar.

Kullanım:
    from planners import get_planner, PLANNER_TYPES
    from planners import get_local_planner, LOCAL_PLANNER_TYPES
    from planners import AStar, Dijkstra, DStarLite, RRT, RRTStar
    from planners import Bug0, Bug1, Bug2, PotentialFields, VFH
"""

from .metrics import get_metric, METRICS
from .base_planner import BasePlanner, CRITERIA
from .path_smoother import smooth_path
from .astar import AStar
from .dijkstra import Dijkstra
from .dstar import DStarLite
from .rrt import RRT
from .rrt_star import RRTStar

# Local (reaktif) planlayıcılar
from .local import (
    get_local_planner, LOCAL_PLANNER_TYPES, LOCAL_PLANNER_DESCS,
    Bug0, Bug1, Bug2, PotentialFields, VFH,
)

PLANNER_TYPES: dict = {
    "astar":    AStar,
    "a*":       AStar,
    "dijkstra": Dijkstra,
    "dstar":    DStarLite,
    "d*":       DStarLite,
    "rrt":      RRT,
    "rrt*":     RRTStar,
    "rrtstar":  RRTStar,
}


def get_planner(planner_type: str, grid_map, metric: str = "euclidean",
                criteria: str = "shortest", cost_map=None,
                **kwargs) -> BasePlanner:
    """
    İsme göre planlayıcı örneği oluştur.

    Args:
        planner_type: 'astar' | 'dijkstra' | 'dstar' | 'rrt' | 'rrt*'
        grid_map:     GridMap örneği.
        metric:       Uzaklık metriği adı.
        criteria:     Yol kriteri ('shortest'|'safest'|'fastest'|'smoothest').
        cost_map:     Clearance mesafe haritası (safest için).
        **kwargs:     Planlayıcıya özgü parametreler.
    Returns:
        BasePlanner alt sınıf örneği.
    Raises:
        ValueError: Geçersiz planlayıcı tipi.
    """
    key = planner_type.lower().strip()
    if key not in PLANNER_TYPES:
        raise ValueError(
            f"Bilinmeyen planlayıcı: '{planner_type}'. "
            f"Geçerli seçenekler: {list(PLANNER_TYPES.keys())}"
        )
    cls = PLANNER_TYPES[key]
    # criteria/cost_map sadece AStar ve Dijkstra tarafından desteklenir
    import inspect
    sig = inspect.signature(cls.__init__)
    if "criteria" in sig.parameters:
        return cls(grid_map, metric=metric, criteria=criteria,
                   cost_map=cost_map, **kwargs)
    return cls(grid_map, metric=metric, **kwargs)


__all__ = [
    "BasePlanner", "CRITERIA",
    "AStar", "Dijkstra", "DStarLite", "RRT", "RRTStar",
    "PLANNER_TYPES", "get_planner",
    "METRICS", "get_metric",
]
