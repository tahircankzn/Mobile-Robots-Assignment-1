"""
planners/metrics.py
Seçilebilir uzaklık metrikleri.

Desteklenen metrikler:
    euclidean  : L2 normu — √(Δx²+Δy²)
    manhattan  : L1 normu — |Δx|+|Δy|
    chebyshev  : L∞ normu — max(|Δx|,|Δy|)
    octile     : 8-yönlü grid için optimize edilmiş
    minkowski  : genel p-norm (p parametresi ile)
    diagonal   : octile ile aynı, başka adla
"""

import numpy as np
from typing import Tuple, Callable


# ------------------------------------------------------------------
# Metrik fonksiyonları
# ------------------------------------------------------------------

def euclidean(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return np.hypot(b[0] - a[0], b[1] - a[1])


def manhattan(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return abs(b[0] - a[0]) + abs(b[1] - a[1])


def chebyshev(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return max(abs(b[0] - a[0]), abs(b[1] - a[1]))


def octile(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    dx = abs(b[0] - a[0])
    dy = abs(b[1] - a[1])
    return max(dx, dy) + (np.sqrt(2) - 1) * min(dx, dy)


def minkowski(p: float = 2):
    """p-norm üreteci (kısmi uygulama)."""
    def _dist(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return (abs(b[0] - a[0]) ** p + abs(b[1] - a[1]) ** p) ** (1 / p)
    _dist.__name__ = f"minkowski_p{p}"
    return _dist


# ------------------------------------------------------------------
# Metrik kataloğu ve fabrika fonksiyonu
# ------------------------------------------------------------------

METRICS: dict[str, Callable] = {
    "euclidean":  euclidean,
    "manhattan":  manhattan,
    "chebyshev":  chebyshev,
    "octile":     octile,
    "diagonal":   octile,           # takma ad
    "minkowski2": minkowski(2),     # Euclidean eşdeğeri
    "minkowski3": minkowski(3),
}


def get_metric(name: str) -> Callable:
    """
    İsme göre metrik fonksiyonu döndür.

    Args:
        name: Metrik adı (büyük/küçük harf duyarsız).
    Returns:
        f(a, b) -> float fonksiyonu.
    Raises:
        ValueError: Geçersiz metrik adı.
    """
    key = name.lower().strip()
    if key not in METRICS:
        raise ValueError(
            f"Bilinmeyen metrik: '{name}'. "
            f"Geçerli seçenekler: {list(METRICS.keys())}"
        )
    return METRICS[key]


__all__ = [
    "euclidean", "manhattan", "chebyshev", "octile", "minkowski",
    "METRICS", "get_metric",
]
