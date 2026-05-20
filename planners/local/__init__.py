"""
planners/local/__init__.py
Local (reaktif) yol planlayıcı kütüphanesi.

Tüm local planlayıcıları ve kayıt sözlüğünü dışa aktarır.

Kullanım:
    from planners.local import get_local_planner, LOCAL_PLANNER_TYPES
    from planners.local import Bug0, Bug1, Bug2, PotentialFields, VFH
"""

from .base_local import BaseLocal, angle_diff
from .bug0 import Bug0
from .bug1 import Bug1
from .bug2 import Bug2
from .potential_fields import PotentialFields
from .vfh import VFH

# ─── Kayıt sözlüğü ──────────────────────────────────────────────────────────
LOCAL_PLANNER_TYPES: dict = {
    "bug0":           Bug0,
    "bug1":           Bug1,
    "bug2":           Bug2,
    "potentialfields": PotentialFields,
    "vfh":            VFH,
}

# Kullanıcıya gösterilen açıklamalar
LOCAL_PLANNER_DESCS: dict = {
    "bug0":           "Reaktif sınır takibi — en basit Bug varyantı",
    "bug1":           "Tam çevreleme → min-mesafe ayrılma noktası",
    "bug2":           "M-line tabanlı — Bug1'den genellikle daha hızlı",
    "potentialfields": "Çekim + İtme kuvveti alanı — akıcı hareket",
    "vfh":            "Polar engel histogramı — dar geçitlerde en iyi",
}


def get_local_planner(planner_type: str) -> BaseLocal:
    """
    İsme göre local planlayıcı örneği oluştur.

    Args:
        planner_type: 'bug0' | 'bug1' | 'bug2' | 'potentialfields' | 'vfh'
    Returns:
        BaseLocal alt sınıf örneği.
    Raises:
        ValueError: Geçersiz planlayıcı tipi.
    """
    key = planner_type.lower().strip()
    if key not in LOCAL_PLANNER_TYPES:
        valid = list(LOCAL_PLANNER_TYPES.keys())
        raise ValueError(
            f"Bilinmeyen local planlayıcı: '{planner_type}'. "
            f"Geçerli seçenekler: {valid}"
        )
    return LOCAL_PLANNER_TYPES[key]()


__all__ = [
    "BaseLocal", "angle_diff",
    "Bug0", "Bug1", "Bug2", "PotentialFields", "VFH",
    "LOCAL_PLANNER_TYPES", "LOCAL_PLANNER_DESCS",
    "get_local_planner",
]
