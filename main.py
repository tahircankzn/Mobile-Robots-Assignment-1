"""
main.py
LiDAR Tabanlı Otonom Navigasyon Simülasyonu — Giriş Noktası

Kullanım:
    python main.py                          # varsayılan ayarlarla
    python main.py --robot differential     # robot tipi seç
    python main.py --planner rrt*           # planlayıcı seç
    python main.py --metric manhattan       # uzaklık metriği seç
    python main.py --list                   # seçenekleri listele

Etkileşimli mod (argüman verilmezse menü açılır):
    python main.py --interactive
"""

import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robots import ROBOT_TYPES
from planners import PLANNER_TYPES, LOCAL_PLANNER_TYPES, METRICS, CRITERIA


def list_options():
    print("\n=== Mevcut Seçenekler ===\n")
    print("Robot tipleri (--robot):")
    for k, cls in ROBOT_TYPES.items():
        print(f"  {k:<15} → {cls.__name__}")

    print("\nYol planlayıcılar (--planner):")
    seen = set()
    for k, cls in PLANNER_TYPES.items():
        if cls not in seen:
            print(f"  {k:<15} → {cls.__name__}")
            seen.add(cls)

    print("\nMesafe metrikleri (--metric):")
    for k in METRICS:
        print(f"  {k}")

    print("\nYol kriterleri (--criteria):")
    for k, desc in CRITERIA.items():
        print(f"  {k:<12} — {desc}")
    print()


def interactive_menu() -> dict:
    """Kullanıcıdan terminal üzerinden seçim al."""
    print("\n" + "="*55)
    print("  LiDAR Tabanlı Otonom Navigasyon — Kurulum Menüsü")
    print("="*55)

    # Robot seçimi
    robots = list(ROBOT_TYPES.keys())
    print("\nRobot tipi seçin:")
    for i, r in enumerate(robots):
        print(f"  [{i+1}] {r}")
    while True:
        try:
            idx = int(input(f"Seçim (1-{len(robots)}): ")) - 1
            if 0 <= idx < len(robots):
                robot = robots[idx]
                break
        except ValueError:
            pass
        print("Geçersiz seçim.")

    # Planlayıcı seçimi
    planners_unique = []
    seen = set()
    for k, cls in PLANNER_TYPES.items():
        if cls not in seen:
            planners_unique.append(k)
            seen.add(cls)

    print("\nYol planlayıcı seçin:")
    for i, p in enumerate(planners_unique):
        print(f"  [{i+1}] {p}")
    while True:
        try:
            idx = int(input(f"Seçim (1-{len(planners_unique)}): ")) - 1
            if 0 <= idx < len(planners_unique):
                planner = planners_unique[idx]
                break
        except ValueError:
            pass
        print("Geçersiz seçim.")

    # Metrik seçimi
    metrics = list(METRICS.keys())
    print("\nUzaklık metriği seçin:")
    for i, m in enumerate(metrics):
        print(f"  [{i+1}] {m}")
    while True:
        try:
            idx = int(input(f"Seçim (1-{len(metrics)}): ")) - 1
            if 0 <= idx < len(metrics):
                metric = metrics[idx]
                break
        except ValueError:
            pass
        print("Geçersiz seçim.")

    print(f"\n→ Robot: {robot} | Planlayıcı: {planner} | Metrik: {metric}\n")
    return {"robot": robot, "planner": planner, "metric": metric,
            "criteria": "shortest", "padding": 1}


def main():
    parser = argparse.ArgumentParser(
        description="LiDAR Tabanlı Otonom Navigasyon Simülasyonu"
    )
    parser.add_argument("--robot",       default="differential")
    parser.add_argument("--planner",     default="astar")
    parser.add_argument("--metric",      default="euclidean")
    parser.add_argument("--criteria",    default="shortest")
    parser.add_argument("--padding",     type=int, default=1)
    parser.add_argument("--dt",          type=float, default=0.05)
    parser.add_argument("--cell-size",   type=float, default=0.5)
    parser.add_argument("--list",        action="store_true")
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--assignment",  action="store_true",
                        help="Headless ödev modu (tüm kombinasyonları koştur)")
    args = parser.parse_args()

    if args.list:
        list_options()
        return

    # ── Pygame penceresi (tek seferlik, geçişler arası kapanmaz) ───────────────
    import pygame
    pygame.init()
    _di = pygame.display.Info()
    _screen = pygame.display.set_mode((_di.current_w, _di.current_h),
                                       pygame.NOFRAME)

    # ── Ana menü döngüsü ────────────────────────────────────────────────────
    while True:
        cfg_from_menu = False
        try:
            from visualization.menu import run_menu
            cfg = run_menu(screen=_screen)
            if cfg is None:
                break
            cfg_from_menu = True
        except Exception as e:
            print(f"Pygame menüsü açılamadı ({e}), CLI moduyla devam ediliyor.")
            cfg = None

        # Menüden cfg gelmediyse CLI / interactive kullan
        if cfg is None:
            if args.interactive:
                cfg = interactive_menu()
            else:
                cfg = {
                    "robot":    args.robot,
                    "planner":  args.planner,
                    "metric":   args.metric,
                    "criteria": args.criteria,
                    "padding":  args.padding,
                    "mode":     "assignment" if args.assignment else "sim",
                }

        # ── Doğrulama ────────────────────────────────────────────────
        robot_type   = cfg.get("robot",    "differential")
        planner_type = cfg.get("planner",  "astar")
        metric       = cfg.get("metric",   "euclidean")
        criteria     = cfg.get("criteria", "shortest")
        padding      = cfg.get("padding",  1)
        mode         = cfg.get("mode",     "sim")

        if robot_type.lower() not in ROBOT_TYPES:
            print(f"Hata: Bilinmeyen robot tipi '{robot_type}'.")
            list_options()
            if not cfg_from_menu:
                sys.exit(1)
            continue
        _pkey = planner_type.lower().strip()
        _is_local = _pkey in LOCAL_PLANNER_TYPES
        if _pkey not in PLANNER_TYPES and not _is_local:
            print(f"Hata: Bilinmeyen planlayıcı '{planner_type}'.")
            list_options()
            if not cfg_from_menu:
                sys.exit(1)
            continue
        if not _is_local and metric.lower() not in METRICS:
            print(f"Hata: Bilinmeyen metrik '{metric}'.")
            list_options()
            if not cfg_from_menu:
                sys.exit(1)
            continue
        if not _is_local and criteria.lower() not in CRITERIA:
            print(f"Hata: Bilinmeyen kriter '{criteria}'.")
            list_options()
            if not cfg_from_menu:
                sys.exit(1)
            continue

        # ── Mod'a göre koştur ────────────────────────────────────────
        if mode == "assignment":
            from assignment_runner import run_assignment
            run_assignment(cfg)
            break
        elif mode == "report":
            from assignment_runner import run_report
            run_report(cfg)
            if not cfg_from_menu:
                break
            # Menüden geldiyse menüye geri dön
        else:
            # ── Tekrar (replay) döngüsü ──────────────────────────────
            action = "replay"
            while action == "replay":
                from simulation import Simulation
                sim = Simulation(
                    robot_type   = robot_type,
                    planner_type = planner_type,
                    metric       = metric,
                    criteria     = criteria,
                    padding      = padding,
                    dt           = args.dt,
                    cell_size    = args.cell_size,
                    screen       = _screen,
                    sensor_params= {k: cfg[k] for k in
                                    ("lidar_range","lidar_beams","lidar_noise")
                                    if k in cfg},
                )
                result = sim.run()
                if isinstance(result, dict):
                    action       = result.get("action", "menu")
                    robot_type   = result.get("robot",    robot_type)
                    planner_type = result.get("planner",  planner_type)
                    metric       = result.get("metric",   metric)
                    criteria     = result.get("criteria", criteria)
                else:
                    action = result or "menu"

            if action == "quit":
                break
            if not cfg_from_menu:
                # CLI modunda menü döngüsü yok
                break
            # action == "menu": dış döngüye dön (menü tekrar açılır)

    pygame.quit()


if __name__ == "__main__":
    main()
