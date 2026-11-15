#!/usr/bin/env python3
"""Plot a SIL CSV trace. Without matplotlib it prints a text summary instead.

    python tools/plot_telemetry.py build/waypoint.csv
    python tools/plot_telemetry.py build/hover.csv --save build/hover.png
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def load(path: Path) -> dict[str, list[float]]:
    columns: dict[str, list[float]] = {}
    with path.open("r", encoding="utf-8", newline="") as fp:
        reader = csv.DictReader(fp)
        if not reader.fieldnames:
            raise ValueError("empty CSV")
        for name in reader.fieldnames:
            columns[name.strip()] = []
        for row in reader:
            for key, value in row.items():
                if key is None:
                    continue
                try:
                    columns[key.strip()].append(float(value))
                except (TypeError, ValueError):
                    columns[key.strip()].append(float("nan"))
    return columns


def summary(c: dict[str, list[float]]) -> None:
    def stat(name: str) -> str:
        vals = [v for v in c.get(name, []) if math.isfinite(v)]
        if not vals:
            return f"{name:>12}: n/a"
        return f"{name:>12}: min {min(vals):8.3f}  max {max(vals):8.3f}  last {vals[-1]:8.3f}"

    print("--- trace summary ---")
    for name in ("t_s", "alt_m", "alt_sp_m", "pos_n_m", "pos_e_m",
                 "roll_rad", "pitch_rad", "yaw_rad", "thrust"):
        print(stat(name))
    a = c.get("alt_m", [])
    sp = c.get("alt_sp_m", [])
    if a and sp:
        errs = [abs(x - y) for x, y in zip(a, sp) if math.isfinite(x) and math.isfinite(y)]
        if errs:
            print(f"{'alt error':>12}: max {max(errs):.3f} m  mean {sum(errs)/len(errs):.3f} m")


def plot(c: dict[str, list[float]], save: str | None) -> None:
    import matplotlib.pyplot as plt

    t = c["t_s"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    axes[0].plot(t, c["alt_m"], label="altitude", linewidth=1.4)
    axes[0].plot(t, c["alt_sp_m"], "--", label="setpoint", linewidth=1.2)
    axes[0].set_ylabel("altitude [m]")
    axes[0].legend(loc="upper right")
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, c["pos_n_m"], label="North", linewidth=1.3)
    axes[1].plot(t, c["pos_e_m"], label="East", linewidth=1.3)
    axes[1].plot(t, c["pos_n_sp_m"], "--", color="tab:blue", alpha=0.6)
    axes[1].plot(t, c["pos_e_sp_m"], "--", color="tab:orange", alpha=0.6)
    axes[1].set_ylabel("position [m]")
    axes[1].legend(loc="upper right")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t, [math.degrees(v) for v in c["roll_rad"]], label="roll", linewidth=1.0)
    axes[2].plot(t, [math.degrees(v) for v in c["pitch_rad"]], label="pitch", linewidth=1.0)
    for i in range(4):
        axes[2].plot(t, c[f"m{i}"], label=f"m{i}", alpha=0.45, linewidth=0.8)
    axes[2].set_ylabel("angle [deg] / motor [0-1]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend(ncol=3, fontsize=8, loc="upper right")
    axes[2].grid(alpha=0.3)

    fig.suptitle("UAV FCS software-in-the-loop trace")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=140)
        print(f"saved {save}")
    else:
        plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path, help="SIL CSV trace")
    ap.add_argument("--save", help="write PNG instead of opening a window")
    ap.add_argument("--no-plot", action="store_true", help="text summary only")
    args = ap.parse_args()

    if not args.csv.exists():
        print(f"file not found: {args.csv}")
        return 2
    cols = load(args.csv)
    summary(cols)
    if args.no_plot:
        return 0
    try:
        plot(cols, args.save)
    except ImportError:
        print("matplotlib not installed - install it with: pip install matplotlib")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
