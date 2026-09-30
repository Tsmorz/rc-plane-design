"""Compare candidate airfoils at the wing's actual Reynolds numbers.

python scripts/01_compare_airfoils.py
python scripts/01_compare_airfoils.py --airfoils sd7062 e387 mh32 --speed 8
"""

from __future__ import annotations

import argparse
import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from planedesign import DESIGN
from planedesign.airfoils import polar, reynolds, summary

warnings.filterwarnings("ignore", category=FutureWarning)
OUT = Path(__file__).resolve().parents[1] / "outputs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--airfoils", nargs="+", default=["sd7062", "sd7037", "e387", "mh32", "clarky"]
    )
    ap.add_argument("--speed", type=float, default=9.0, help="cruise speed [m/s]")
    args = ap.parse_args()

    w = DESIGN.wing
    re_root = reynolds(args.speed, w.root_chord, DESIGN.field_altitude_m)
    re_tip = reynolds(args.speed, w.tip_chord, DESIGN.field_altitude_m)
    print(f"Re at {args.speed} m/s: root {re_root:,.0f}, tip {re_tip:,.0f}\n")

    header = f"{'airfoil':<9}{'Re':>8}{'t/c':>6}{'CLmax':>7}{'L/Dmax':>8}{'CL^1.5/CD':>11}{'CM@0.5':>8}{'conf':>6}"
    print(header)
    print("-" * len(header))
    for name in args.airfoils:
        for re in (re_root, re_tip):
            s = summary(name, re)
            print(
                f"{name:<9}{re / 1000:7.0f}k{s['t/c']:6.3f}{s['CLmax']:7.2f}{s['L/D_max']:8.1f}"
                f"{s['CL^1.5/CD_max']:11.1f}{s['CM_at_CL0.5']:8.3f}{s['min_confidence']:6.2f}"
            )
    print("\nconf = NeuralFoil confidence (<0.9: treat that polar with suspicion)")
    print(
        "CL^1.5/CD is the endurance figure of merit; CM matters for tail size/trim drag."
    )

    fig, axs = plt.subplots(1, 3, figsize=(16, 5))
    for name in args.airfoils:
        p = polar(name, re_tip)
        (line,) = axs[0].plot(p["CD"], p["CL"], label=name)
        axs[1].plot(p["alpha"], p["CL"], color=line.get_color())
        axs[2].plot(p["alpha"], p["CM"], color=line.get_color())
    axs[0].set(
        xlabel="CD", ylabel="CL", title=f"Drag polar @ tip Re {re_tip / 1000:.0f}k"
    )
    axs[1].set(xlabel="alpha [deg]", ylabel="CL", title="Lift curve")
    axs[2].set(xlabel="alpha [deg]", ylabel="CM", title="Pitching moment")
    for a in axs:
        a.grid(alpha=0.3)
    axs[0].legend()
    fig.tight_layout()
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / "airfoil_comparison.png", dpi=130)
    print(f"\nplot: {OUT / 'airfoil_comparison.png'}")


if __name__ == "__main__":
    main()
