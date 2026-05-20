"""
planners/local/vfh.py
Vector Field Histogram (VFH) algoritması.

Adımlar:
    1. LiDAR ışınlarından polar engel histogramı oluştur.
       Her sektör: engel yoğunluğu (0 = tamamen açık, 1 = tamamen dolu).
    2. Eşik uygula → sektörler "engelli" veya "serbest" olarak işaretlenir.
    3. Serbest sektör gruplarını (valley) bul.
    4. Hedefe en yakın serbest vadiyi seç.
    5. Seçilen sektöre doğru yönel.

Özellik:
    - Durumsuz (tamamen reaktif).
    - Dar geçitlerde en iyi performans gösteren lokal planlayıcı.
    - Potansiyel alanların aksine yerel minimumlara çok daha az takılır.
    - Engel sinyalinin 2D yumuşatılması ile gürültüye karşı dayanıklıdır.
"""

import numpy as np
from .base_local import BaseLocal, angle_diff


# ─── Parametre sabitleri ─────────────────────────────────────────────────────
_NUM_SECTORS  = 72         # 360° / 5° = 72 sektör
_SECTOR_SIZE  = 2.0 * np.pi / _NUM_SECTORS  # radyan / sektör
_THRESHOLD    = 0.40       # engel yoğunluğu eşiği (0-1)
_VALLEY_MIN   = 2          # minimum serbest vadi genişliği (sektör sayısı)
_LIDAR_MAX_R  = 8.0        # varsayılan LiDAR max mesafe (m) — histogram normalize için
_MAX_V        = 1.2        # maksimum doğrusal hız (m/s)
_MAX_OMEGA    = 1.8        # maksimum açısal hız (rad/s)
_SAFE_DIST    = 1.8        # bu mesafeden yakın engel varsa yavaşla (m)
_SPREAD_W     = 0.35       # histogram komşu sektör yayılım katsayısı


class VFH(BaseLocal):
    """
    Vector Field Histogram: Polar engel yoğunluğu → serbest vadi → yönlendirme.

    Durumsuz — her adımda sadece anlık LiDAR ve hedef kullanılır.
    """

    name = "VFH"

    def reset(self, start_x, start_y, goal_x, goal_y):
        pass  # Durumsuz algoritma

    # ─── Histogram oluşturma ─────────────────────────────────────────────────

    def _build_histogram(self, ranges, beam_angles, rtheta):
        """
        LiDAR verilerinden robot-göreceli polar engel histogramı oluşturur.

        Sektör 0 = robot ilerisi (θ=0), değerler [0, 2π) yönünde artar.
        Her sektör değeri = [0, 1] arası normalize engel yoğunluğu.
        """
        hist    = np.zeros(_NUM_SECTORS)
        max_r   = max(float(np.max(ranges)), 0.1) if len(ranges) else _LIDAR_MAX_R

        for ang, d in zip(beam_angles, ranges):
            # Robot-göreceli açıyı [0, 2π) aralığına getir
            ang_norm   = float(ang) % (2.0 * np.pi)
            sector_idx = int(ang_norm / _SECTOR_SIZE) % _NUM_SECTORS

            # Yakın engel = yüksek katkı (ikinci dereceden ağırlık)
            contrib = max(0.0, (max_r - d) / max_r) ** 2

            # Ana sektör + komşulara yayılım (robot boyutu güvenlik payı)
            for offset, weight in ((-1, _SPREAD_W), (0, 1.0), (1, _SPREAD_W)):
                idx = (sector_idx + offset) % _NUM_SECTORS
                hist[idx] = min(1.0, hist[idx] + contrib * weight)

        return hist

    # ─── Vadi bulma ──────────────────────────────────────────────────────────

    def _find_valleys(self, blocked):
        """
        Histogram'daki serbest sektör gruplarını (valley) döndürür.

        Dairesel yapıyı doğru ele almak için ikili geçiş kullanılır.
        Her vadi: (start_sector, length) çifti.
        """
        n       = _NUM_SECTORS
        valleys = []

        # Dairesel tarama: başlangıç noktasını engelli bir sektörden başlat
        start = 0
        for i in range(n):
            if blocked[i]:
                start = (i + 1) % n
                break

        in_valley    = False
        valley_start = 0
        valley_len   = 0

        for k in range(n + 1):
            i = (start + k) % n
            if not blocked[i]:
                if not in_valley:
                    in_valley    = True
                    valley_start = i
                    valley_len   = 0
                valley_len += 1
            else:
                if in_valley:
                    in_valley = False
                    if valley_len >= _VALLEY_MIN:
                        valleys.append((valley_start, valley_len))

        if in_valley and valley_len >= _VALLEY_MIN:
            valleys.append((valley_start, valley_len))

        return valleys

    # ─── Ana hesaplama ───────────────────────────────────────────────────────

    def compute(self, rx, ry, rtheta, ranges, beam_angles, gx, gy):
        goal_angle = np.arctan2(gy - ry, gx - rx)
        dist_goal  = np.hypot(gx - rx, gy - ry)

        # ── 1. Polar histogramı oluştur ───────────────────────────────────────
        hist    = self._build_histogram(ranges, beam_angles, rtheta)
        blocked = hist > _THRESHOLD

        # ── 2. Hedef sektörünü hesapla (robot-göreceli çerçeve) ───────────────
        goal_rel      = angle_diff(goal_angle, rtheta)      # [-π, π]
        goal_rel_norm = float(goal_rel) % (2.0 * np.pi)    # [0, 2π)
        goal_sector   = int(goal_rel_norm / _SECTOR_SIZE) % _NUM_SECTORS

        # ── 3. Hedef sektörü serbest mi? Doğrudan git ─────────────────────────
        if not blocked[goal_sector]:
            chosen_sector = goal_sector
        else:
            # ── 4. En yakın serbest vadiyi bul ───────────────────────────────
            valleys = self._find_valleys(blocked)

            if not valleys:
                # Tüm sektörler engelli → hızı düşür, yerinde hafifçe dön
                return 0.1, 0.8

            # Hedef sektörüne dairesel mesafece en yakın vadi merkezini seç
            best_sector = None
            best_diff   = float("inf")

            for v_start, v_len in valleys:
                center = (v_start + v_len // 2) % _NUM_SECTORS
                diff   = abs(center - goal_sector)
                diff   = min(diff, _NUM_SECTORS - diff)   # dairesel mesafe
                if diff < best_diff:
                    best_diff   = diff
                    best_sector = center

            chosen_sector = best_sector

        # ── 5. Seçilen sektöre doğru yönel ───────────────────────────────────
        chosen_norm = chosen_sector * _SECTOR_SIZE          # [0, 2π)
        # Robot-göreceli açıya dönüştür: [0, 2π) → [-π, π]
        theta_err   = (chosen_norm + np.pi) % (2.0 * np.pi) - np.pi

        # ── 6. Hız: ön sektör boşluğuna göre ayarla ──────────────────────────
        fwd_mask = np.abs(beam_angles) < np.pi / 4
        min_fwd  = float(np.min(ranges[fwd_mask])) if fwd_mask.any() else _LIDAR_MAX_R
        speed_factor = np.clip((min_fwd - 0.3) / _SAFE_DIST, 0.1, 1.0)

        v     = np.clip(_MAX_V * speed_factor, 0.0, _MAX_V)
        omega = np.clip(2.2 * theta_err, -_MAX_OMEGA, _MAX_OMEGA)

        return v, omega
