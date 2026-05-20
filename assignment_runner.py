"""
assignment_runner.py
Ödev modu: tüm parametre kombinasyonlarını headless çalıştırır
ve her biri için PNG kaydeder.

Kullanım:
    from assignment_runner import run_assignment
    run_assignment(cfg)   # cfg dict'i menüden gelir

veya doğrudan:
    python assignment_runner.py
"""

from __future__ import annotations
import os
import time
import itertools
import numpy as np
import traceback
from datetime import datetime

# ─── Senaryo Matrisi ──────────────────────────────────────────────────
ROBOTS    = ["differential", "ackermann", "omni", "mecanum"]
PLANNERS  = ["astar", "dijkstra", "rrt", "rrtstar"]
METRICS   = ["euclidean", "manhattan", "chebyshev"]
CRITERIA  = ["shortest", "safest", "fastest", "smoothest"]

# RRT tabanlı planlayıcılar metrik kullanmaz; sadece "shortest" mantıklı
RRT_PLANNERS = {"rrt", "rrtstar", "dstar"}


def _build_scenarios(robot_filter=None, planner_filter=None,
                     metric_filter=None, criteria_filter=None):
    """Filtreli senaryo listesi üret."""
    robots   = [robot_filter]   if robot_filter   else ROBOTS
    planners = [planner_filter] if planner_filter  else PLANNERS
    metrics  = [metric_filter]  if metric_filter   else METRICS
    criterias = [criteria_filter] if criteria_filter else CRITERIA

    scenarios = []
    for robot, planner, metric, criteria in itertools.product(
            robots, planners, metrics, criterias):
        # RRT'de metrik değişkeni anlamsız — sadece euclidean al
        if planner in RRT_PLANNERS and metric != "euclidean":
            continue
        scenarios.append({
            "robot":    robot,
            "planner":  planner,
            "metric":   metric,
            "criteria": criteria,
        })
    return scenarios


def _tag(s: dict) -> str:
    return f"{s['robot']}_{s['planner']}_{s['metric']}_{s['criteria']}"


def run_assignment(cfg: dict | None = None,
                   output_dir: str | None = None,
                   padding: int = 1):
    """
    Tüm senaryoları headless çalıştır; PNG ve metin çıktıları zaman damgalı
    klasöre kaydet.

    Args:
        cfg:        Menüden gelen dict (filtreler için kullanılır) veya None.
        output_dir: Çıktı klasörü; None → outputs/odev_YYYYMMDD_HHMMSS/
        padding:    Engel dolgusu (hücre sayısı).
    """
    from simulation import Simulation

    if output_dir is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join("outputs", f"odev_{ts}")

    os.makedirs(output_dir, exist_ok=True)

    # Menü filtresi uygulanırsa tek robot/planner/metric/criteria sabitlenerek
    # yalnızca diğerleri değiştirilir — ya da tümü döner.
    robot_f    = cfg.get("robot")    if cfg else None
    planner_f  = cfg.get("planner")  if cfg else None
    metric_f   = cfg.get("metric")   if cfg else None
    criteria_f = cfg.get("criteria") if cfg else None
    if cfg:
        padding = cfg.get("padding", padding)

    scenarios = _build_scenarios(robot_f, planner_f, metric_f, criteria_f)
    total     = len(scenarios)

    print(f"\n{'='*58}")
    print(f"  ÖDEV MODU — {total} senaryo çalıştırılacak")
    print(f"  Çıktı klasörü : {output_dir}")
    print(f"  Engel dolgusu : {padding} hücre ({padding * 0.5:.1f} m)")
    print(f"{'='*58}\n")

    results = []
    t_start = time.time()

    for idx, sc in enumerate(scenarios, 1):
        tag  = _tag(sc)
        tag_short = tag.replace("differential", "diff")
        print(f"[{idx:>3}/{total}]  {tag_short}  ", end="", flush=True)

        try:
            sim = Simulation(
                robot_type   = sc["robot"],
                planner_type = sc["planner"],
                metric       = sc["metric"],
                criteria     = sc["criteria"],
                padding      = padding,
                headless     = True,
                output_tag   = os.path.relpath(output_dir, "outputs"),
            )

            t0  = time.time()
            sim.run()
            elapsed = time.time() - t0

            errs = np.array(sim.position_errors)
            rmse = float(np.sqrt(np.mean(errs**2))) if len(errs) else 0.0
            plen = sim.path_stats.get("path_length", 0.0)
            print(f"OK  RMSE={rmse:.3f}m  plen={plen:.1f}  t={elapsed:.1f}s")
            results.append({**sc, "rmse": rmse, "path_len": plen,
                             "time": elapsed, "ok": True})

        except Exception as e:
            print(f"HATA: {e}")
            results.append({**sc, "rmse": None, "path_len": None,
                             "time": None, "ok": False})

    # ── Özet tablosu ──────────────────────────────────────────────────
    elapsed_total = time.time() - t_start
    ok_count      = sum(1 for r in results if r["ok"])
    print(f"\n{'='*58}")
    print(f"  Tamamlandı: {ok_count}/{total}  (toplam {elapsed_total:.1f}s)")

    _save_summary(results, output_dir)
    _save_summary_text(results, output_dir)
    abs_path = os.path.abspath(output_dir)
    print(f"  Özet PNG    → {output_dir}/ozet.png")
    print(f"  Sonuç TXT   → {output_dir}/SONUCLAR.txt")
    print(f"  Alt klasör  → {output_dir}/<robot>__<planlayıcı>__<metrik>__<kriter>/")
    print(f"\n  *** ÖDEV ÇIKTILARI: {abs_path} ***")
    print(f"{'='*58}\n")
    return results


def _save_summary_text(results: list, output_dir: str,
                       fname: str = "SONUCLAR.txt",
                       title: str = "ÖDEV MODU — SİMÜLASYON SONUÇLARI"):
    """Tüm sonuçları tablo formatında metin dosyasına kaydet."""
    path = os.path.join(output_dir, fname)
    ok_results = [r for r in results if r["ok"]]
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write("=" * 80 + "\n")
            f.write(f"  {title}\n")
            f.write(f"  Tarih : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"  Klasör: {os.path.abspath(output_dir)}\n")
            f.write("=" * 80 + "\n\n")

            hdr = (f"{'Robot':<16} {'Planlay\u0131c\u0131':<12} {'Metrik':<12}"
                   f" {'Kriter':<12} {'RMSE(m)':<10} {'Yol(hcr)':<10}"
                   f" {'S\u00fcre(s)':<9} Durum")
            f.write(hdr + "\n")
            f.write("-" * 85 + "\n")
            for r in results:
                rmse = f"{r['rmse']:.4f}"     if r['rmse']     is not None else "HATA"
                plen = f"{r['path_len']:.1f}" if r['path_len'] is not None else "-"
                t    = f"{r['time']:.1f}"     if r['time']     is not None else "-"
                ok   = "OK" if r['ok'] else "HATA"
                f.write(
                    f"{r['robot']:<16} {r['planner']:<12} {r['metric']:<12}"
                    f" {r['criteria']:<12} {rmse:<10} {plen:<10} {t:<9} {ok}\n"
                )

            f.write("\n" + "=" * 80 + "\n")
            f.write(f"  Toplam: {len(results)}  |  Başarılı: {len(ok_results)}"
                    f"  |  Başarısız: {len(results) - len(ok_results)}\n")
            if ok_results:
                best_rmse = min(ok_results, key=lambda r: r['rmse'])
                best_plen = min(ok_results, key=lambda r: r['path_len'])
                f.write(f"\n  En Düşük RMSE : {best_rmse['robot']} / {best_rmse['planner']}"
                        f" / {best_rmse['metric']} / {best_rmse['criteria']}"
                        f" = {best_rmse['rmse']:.4f} m\n")
                f.write(f"  En Kısa Yol   : {best_plen['robot']} / {best_plen['planner']}"
                        f" / {best_plen['metric']} / {best_plen['criteria']}"
                        f" = {best_plen['path_len']:.1f} hücre\n")
            f.write("=" * 80 + "\n")
    except Exception as e:
        print(f"  SONUCLAR.txt kaydedilemedi: {e}")


def _save_summary(results: list, output_dir: str):
    """Tüm sonuçları karşılaştırmalı özet PNG'sine kaydet."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        ok_results = [r for r in results if r["ok"] and r["rmse"] is not None]
        if not ok_results:
            print("  Özet için başarılı sonuç yok.")
            return

        labels  = [f"{r['planner']}\n{r['metric']}\n{r['criteria']}"
                   for r in ok_results]
        rmses   = [r["rmse"]     for r in ok_results]
        plens   = [r["path_len"] for r in ok_results]
        robots  = sorted(set(r["robot"] for r in ok_results))

        fig, axes = plt.subplots(1, 2, figsize=(max(14, len(ok_results) * 0.6), 6))
        fig.patch.set_facecolor("#0a0f1c")

        colors_map = {
            "differential": "#4488FF",
            "omni":         "#00CCDD",
            "ackermann":    "#FF8822",
            "mecanum":      "#BB44FF",
        }

        for ax, values, title, unit in zip(
                axes,
                [rmses, plens],
                ["RMSE (m)", "Yol Uzunluğu (m)"],
                ["m",        "m"]):
            ax.set_facecolor("#0a0f1c")
            xs = range(len(ok_results))
            bar_colors = [colors_map.get(r["robot"], "#888") for r in ok_results]
            ax.bar(xs, values, color=bar_colors, edgecolor='#445', linewidth=0.5)
            ax.set_xticks(list(xs))
            ax.set_xticklabels(labels, rotation=60, ha='right',
                               fontsize=6, color='#AAC')
            ax.set_ylabel(unit, color='#AAC')
            ax.set_title(title, color='#88CCFF')
            ax.tick_params(axis='y', colors='#AAC')
            for sp in ax.spines.values():
                sp.set_color('#445')
            ax.yaxis.grid(True, alpha=0.2, color='#335')

        # Robot renk lejandı
        from matplotlib.patches import Patch
        legend_elems = [Patch(facecolor=c, label=r)
                        for r, c in colors_map.items() if r in robots]
        fig.legend(handles=legend_elems, loc='upper right',
                   facecolor='#111', labelcolor='white', fontsize=8)

        fig.suptitle("Ödev Modu — Senaryo Karşılaştırması",
                     color='#88CCFF', fontsize=13)
        fig.tight_layout()
        os.makedirs(output_dir, exist_ok=True)
        fig.savefig(os.path.join(output_dir, "ozet.png"), dpi=150)
        plt.close(fig)

    except ImportError:
        print("  matplotlib bulunamadı, özet kaydedilemedi.")
    except Exception as e:
        print(f"  Özet kaydedilemedi: {e}")


# ─── Rapor Modu — Tüm Araçlar İçin PDF Gereksinimlerini Karşılayan Çıktılar ──

# Her robot tipi için en uygun senaryo (1 senaryo/robot)
REPORT_SCENARIOS = [
    # Non-holonomik robotlar (ödev odağı)
    {"robot": "differential", "planner": "astar",    "metric": "euclidean", "criteria": "shortest"},
    {"robot": "ackermann",    "planner": "astar",    "metric": "euclidean", "criteria": "shortest"},
    {"robot": "fixedwing",    "planner": "astar",    "metric": "euclidean", "criteria": "safest"},
    {"robot": "snake",        "planner": "astar",    "metric": "manhattan", "criteria": "shortest"},
    {"robot": "bipedal",      "planner": "astar",    "metric": "euclidean", "criteria": "shortest"},
    # Holonomik robotlar (karşılaştırma)
    {"robot": "omni",         "planner": "astar",    "metric": "chebyshev", "criteria": "shortest"},
    {"robot": "mecanum",      "planner": "dijkstra", "metric": "manhattan", "criteria": "safest"},
    {"robot": "quadruped",    "planner": "astar",    "metric": "euclidean", "criteria": "shortest"},
    {"robot": "hexapod",      "planner": "astar",    "metric": "euclidean", "criteria": "shortest"},
    {"robot": "drone",        "planner": "astar",    "metric": "euclidean", "criteria": "shortest"},
    {"robot": "vtol",         "planner": "astar",    "metric": "euclidean", "criteria": "shortest"},
]

_REPORT_NON_HOLO = {"differential", "ackermann", "fixedwing", "snake", "bipedal"}


def run_report(cfg: dict | None = None,
               output_dir: str | None = None,
               padding: int = 1):
    """
    Rapor modu: PDF ödev gereksinimlerini karşılayan tüm çıktıları üretir.
    Her robot tipi için headless simülasyon çalıştırır ve aşağıdaki PNG'leri kaydeder:
      hata.png, yol.png, lidar.png, lokalizasyon.png  (her robot alt klasöründe)
    Ek olarak: ozet_rapor.png ve SONUCLAR_RAPOR.txt (üst klasörde)

    Args:
        cfg:        Menüden gelen dict (padding için kullanılır) veya None.
        output_dir: Çıktı klasörü; None → outputs/rapor_YYYYMMDD_HHMMSS/
        padding:    Engel dolgusu (hücre sayısı).
    """
    from simulation import Simulation

    if output_dir is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join("outputs", f"rapor_{ts}")

    os.makedirs(output_dir, exist_ok=True)

    if cfg:
        padding = cfg.get("padding", padding)

    total = len(REPORT_SCENARIOS)
    print(f"\n{'='*60}")
    print(f"  RAPOR MODU — {total} robot için PDF çıktıları üretiliyor")
    print(f"  Çıktı klasörü : {output_dir}")
    print(f"  Engel dolgusu : {padding} hücre ({padding * 0.5:.1f} m)")
    print(f"  PDF Bölümleri : 6.1 Harita · 6.2 Yol · 6.3 LiDAR · 6.4 Lok. · 6.5 Hata")
    print(f"{'='*60}\n")

    results = []
    t_start = time.time()

    for idx, sc in enumerate(REPORT_SCENARIOS, 1):
        nholo = "(non-holo)" if sc["robot"] in _REPORT_NON_HOLO else "(holonomik)"
        print(f"[{idx:>2}/{total}]  {sc['robot']:<14} {nholo}  ", end="", flush=True)

        try:
            sim = Simulation(
                robot_type   = sc["robot"],
                planner_type = sc["planner"],
                metric       = sc["metric"],
                criteria     = sc["criteria"],
                padding      = padding,
                headless     = True,
                output_tag   = os.path.relpath(output_dir, "outputs"),
            )
            t0 = time.time()
            sim.run()
            elapsed = time.time() - t0

            errs = np.array(sim.position_errors)
            rmse = float(np.sqrt(np.mean(errs**2))) if len(errs) else 0.0
            plen = sim.path_stats.get("path_length", 0.0)
            print(f"OK  RMSE={rmse:.4f}m  yol={plen:.1f}  t={elapsed:.1f}s")
            results.append({**sc, "rmse": rmse, "path_len": plen,
                            "time": elapsed, "ok": True})

        except Exception as e:
            print(f"HATA: {e}")
            traceback.print_exc()
            results.append({**sc, "rmse": None, "path_len": None,
                            "time": None, "ok": False})

    elapsed_total = time.time() - t_start
    ok_count = sum(1 for r in results if r["ok"])
    print(f"\n{'='*60}")
    print(f"  Tamamlandı: {ok_count}/{total}  (toplam {elapsed_total:.1f}s)")

    _save_report_summary(results, output_dir)
    _save_summary_text(results, output_dir, fname="SONUCLAR_RAPOR.txt",
                       title="RAPOR MODU — SİMÜLASYON SONUÇLARI")
    abs_path = os.path.abspath(output_dir)
    print(f"  Özet PNG    → {output_dir}/ozet_rapor.png")
    print(f"  Sonuç TXT   → {output_dir}/SONUCLAR_RAPOR.txt")
    print(f"  Alt klasör  → {output_dir}/<robot>__<planlayıcı>__<metrik>__<kriter>/")
    print(f"\n  *** RAPOR ÇIKTILARI: {abs_path} ***")
    print(f"{'='*60}\n")
    return results


def _save_report_summary(results: list, output_dir: str):
    """Tüm robotları karşılaştıran kapsamlı özet figürü (rapor modu için)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch

        ok = [r for r in results if r["ok"] and r["rmse"] is not None]
        if not ok:
            print("  Özet için başarılı sonuç yok.")
            return

        _colors = {
            "differential": "#4488FF", "ackermann":  "#FF8822",
            "fixedwing":    "#FF4444", "snake":      "#00BBBB",
            "bipedal":      "#BB44FF", "omni":       "#00CCDD",
            "mecanum":      "#AA33FF", "quadruped":  "#44BB44",
            "hexapod":      "#FFAA00", "drone":      "#33CC77",
            "vtol":         "#0099CC",
        }

        robots = [r["robot"]    for r in ok]
        rmses  = [r["rmse"]     for r in ok]
        plens  = [r["path_len"] for r in ok]
        times  = [r["time"]     for r in ok]
        bar_c  = [_colors.get(r, "#888") for r in robots]
        edgec  = ["#FFD700" if r in _REPORT_NON_HOLO else "#334455" for r in robots]

        fig, axes = plt.subplots(3, 1, figsize=(14, 12))
        fig.patch.set_facecolor("#0a0f1c")
        fig.suptitle(
            "Rapor Modu — Robot Karşılaştırması\n"
            "(Sarı kenarlı = Non-holonomik  |  Tüm Araçlar)",
            color='#88CCFF', fontsize=14)

        for ax, vals, title, unit in zip(
                axes,
                [rmses, plens, times],
                ["Lokalizasyon Hatası — EKF RMSE (m)",
                 "Yol Uzunluğu (hücre)",
                 "Simülasyon Süresi (s)"],
                ["m", "hücre", "s"]):
            ax.set_facecolor("#0a0f1c")
            xs = range(len(ok))
            bars = ax.bar(xs, vals, color=bar_c, edgecolor=edgec, linewidth=2.0)
            for bar, val in zip(bars, vals):
                if val is not None:
                    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                            f"{val:.3f}", ha='center', va='bottom',
                            color='white', fontsize=8)
            ax.set_xticks(list(xs))
            ax.set_xticklabels(robots, rotation=30, ha='right',
                               fontsize=10, color='#AAC')
            ax.set_ylabel(unit, color='#AAC')
            ax.set_title(title, color='#88CCFF', fontsize=11)
            ax.tick_params(axis='y', colors='#AAC')
            for sp in ax.spines.values(): sp.set_color('#445')
            ax.yaxis.grid(True, alpha=0.2, color='#335')

        patches = [
            Patch(facecolor=_colors.get(r, "#888"), edgecolor="#FFD700",
                  linewidth=2, label=f"{r} ★")
            for r in _REPORT_NON_HOLO if r in _colors
        ] + [
            Patch(facecolor=_colors.get(r, "#888"), label=r)
            for r in _colors if r not in _REPORT_NON_HOLO
        ]
        fig.legend(handles=patches, loc='upper right',
                   bbox_to_anchor=(0.99, 0.97),
                   facecolor='#111', labelcolor='white',
                   fontsize=9, ncol=2, framealpha=0.9)

        fig.tight_layout(rect=[0, 0, 1, 0.93])
        fig.savefig(os.path.join(output_dir, "ozet_rapor.png"), dpi=150)
        plt.close(fig)
        print(f"  Özet → {output_dir}/ozet_rapor.png")

    except ImportError:
        print("  matplotlib bulunamadı, özet kaydedilemedi.")
    except Exception as e:
        print(f"  Özet kaydedilemedi: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    run_assignment()
