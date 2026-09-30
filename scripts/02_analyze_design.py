"""Analyze the design in config.py: mass budget, CG, static margin, trim, performance.

python scripts/02_analyze_design.py              # analyze as configured
python scripts/02_analyze_design.py --solve-cg   # move the battery to hit the target static margin
"""

from __future__ import annotations

import argparse
import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as onp

from planedesign import DESIGN
from planedesign import analysis as A
from planedesign import mass as M
from planedesign.config import MAX_TAKEOFF_MASS_G
from planedesign.geometry import make_airplane

warnings.filterwarnings("ignore", category=FutureWarning)
OUT = Path(__file__).resolve().parents[1] / "outputs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--solve-cg", action="store_true", help="place battery for target static margin"
    )
    ap.add_argument(
        "--speeds", nargs="+", type=float, default=[7, 8, 9, 10, 11, 12, 14]
    )
    args = ap.parse_args()
    d = DESIGN
    OUT.mkdir(exist_ok=True)

    # --- geometry & stability ---------------------------------------------
    x_np = A.neutral_point(d)
    mac = A.mac(d)
    if args.solve_cg:
        x_target = x_np - d.target_static_margin * mac
        x_bat = M.battery_x_for_cg(d, x_target)
        d = M.with_battery_at(d, x_bat)
        print(f"[solve-cg] battery moved to x = {x_bat * 1000:.0f} mm\n")

    print("=== MASS BUDGET ===")
    print(M.budget_table(d))
    if M.total_mass_g(d) >= MAX_TAKEOFF_MASS_G:
        print("!!! OVER THE 250 g LEGAL LIMIT !!!")

    x_cg = M.cg(d)[0]
    sm = (x_np - x_cg) / mac
    s = A.wing_area(d)
    print("\n=== GEOMETRY & STABILITY ===")
    print(f"wing area        {s * 100:.1f} dm^2")
    print(f"MAC              {mac * 1000:.0f} mm")
    print(f"wing loading     {M.total_mass_g(d) / (s * 100):.1f} g/dm^2")
    print(f"neutral point    {x_np * 1000:.0f} mm behind wing LE")
    print(f"CG               {x_cg * 1000:.0f} mm behind wing LE")
    print(
        f"static margin    {sm * 100:.1f} % MAC  (target {d.target_static_margin * 100:.0f} %)"
    )
    if sm < 0.05:
        print("!!! static margin too small: pitch-unstable or twitchy, move CG forward")
    elif sm > 0.25:
        print(
            "!!! static margin very large: nose-heavy, needs lots of up-elevator, move CG aft"
        )

    # --- performance --------------------------------------------------------
    vs = A.stall_speed(d)
    print("\n=== PERFORMANCE ===")
    print(f"stall speed      {vs:.1f} m/s  (airplane CLmax estimate)")
    print(
        f"\n{'V m/s':>6}{'alpha':>7}{'tail inc':>9}{'CL':>6}{'L/D':>6}{'P_elec W':>9}{'t ideal':>8}{'t real':>7}"
    )
    pts = []
    for v in args.speeds:
        if v < 1.2 * vs:
            continue
        try:
            p = A.trim(d, v)
        except RuntimeError:
            print(f"{v:6.1f}  trim failed")
            continue
        pts.append(p)
        print(
            f"{v:6.1f}{p.alpha_deg:7.1f}{p.tail_incidence_deg:9.1f}{p.CL:6.2f}{p.L_over_D:6.1f}"
            f"{p.electrical_power_W:9.1f}{p.flight_time_min:7.0f}m{p.realistic_flight_time_min:6.0f}m"
        )
    print(
        "\nalpha is fuselage angle of attack; the wing sits at "
        f"+{d.wing.root_incidence_deg:.0f} deg incidence on top of that."
    )
    print(
        "tail inc = stabilizer incidence needed to trim. Build the stab at the value for your"
    )
    print("cruise speed; the change across speeds is what the elevator has to supply.")
    print(
        f"t real assumes {d.real_world_power_factor}x ideal power (climbs, turns, gusts, build drag)."
    )

    # --- plots --------------------------------------------------------------
    if pts:
        v = onp.array([p.velocity for p in pts])
        fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
        axs[0].plot(v, [p.electrical_power_W for p in pts], "o-", label="ideal")
        axs[0].plot(
            v,
            [p.electrical_power_W * d.real_world_power_factor for p in pts],
            "o--",
            label="realistic",
        )
        axs[0].axvline(vs, color="r", ls=":", label="stall")
        axs[0].set(
            xlabel="airspeed [m/s]",
            ylabel="electrical power [W]",
            title="Power required",
        )
        axs[1].plot(v, [p.tail_incidence_deg for p in pts], "o-")
        axs[1].set(
            xlabel="airspeed [m/s]", ylabel="stab incidence to trim [deg]", title="Trim"
        )
        for a in axs:
            a.grid(alpha=0.3)
        axs[0].legend()
        fig.tight_layout()
        fig.savefig(OUT / "performance.png", dpi=130)

    airplane = make_airplane(d, xyz_ref=(x_cg, 0, 0))
    airplane.draw_three_view(show=False)
    plt.gcf().savefig(OUT / "three_view.png", dpi=130)
    print(f"\nplots: {OUT / 'performance.png'}, {OUT / 'three_view.png'}")


if __name__ == "__main__":
    main()
