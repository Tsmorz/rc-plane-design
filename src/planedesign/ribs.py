"""Rib stations for one wing half, in each rib's own flat 2D frame.

Ribs are flat printed parts, so outlines are exported *unrotated*: chord along +x,
leading edge at the origin. Incidence/washout is applied at build time by jig tabs
(generated in the CadQuery step from `twist_deg` and `spar_center`), and dihedral
by joining the two wing halves at the root.

The spar hole sits on the camber line at `spar_x_frac` of chord. Because every rib
is the same airfoil scaled linearly with span, those points lie on a straight line,
so a straight carbon tube passes through all of them.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import aerosandbox as asb
import aerosandbox.numpy as np

from .config import Design


@dataclass
class Rib:
    index: int
    y: float                    # span station from root [m]
    chord: float                # [m]
    twist_deg: float            # incidence at this station (root incidence - washout share)
    outline: np.ndarray         # (N, 2) airfoil coords [m], rib-local frame
    spar_center: tuple[float, float]


def rib_stations(d: Design, spar_x_frac: float = 0.25) -> list[Rib]:
    w = d.wing
    semi = w.span / 2
    n = int(round(semi / d.rib_spacing)) + 1
    af = asb.Airfoil(w.airfoil)
    ribs = []
    for i, y in enumerate(np.linspace(0, semi, n)):
        eta = y / semi
        chord = w.root_chord + eta * (w.tip_chord - w.root_chord)
        spar_z = float(af.local_camber(x_over_c=spar_x_frac)) * chord
        ribs.append(Rib(
            index=i,
            y=float(y),
            chord=float(chord),
            twist_deg=float(w.root_incidence_deg - eta * w.washout_deg),
            outline=af.coordinates * chord,
            spar_center=(spar_x_frac * chord, spar_z),
        ))
    return ribs


def export_csv(ribs: list[Rib], out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = out_dir / "rib_summary.csv"
    with summary.open("w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["rib", "y_mm", "chord_mm", "twist_deg", "spar_x_mm", "spar_z_mm", "outline_file"])
        for r in ribs:
            name = f"rib_{r.index:02d}.csv"
            np.savetxt(out_dir / name, r.outline * 1000, delimiter=",", header="x_mm,z_mm", comments="")
            wr.writerow([r.index, f"{r.y * 1000:.1f}", f"{r.chord * 1000:.1f}", f"{r.twist_deg:.2f}",
                         f"{r.spar_center[0] * 1000:.2f}", f"{r.spar_center[1] * 1000:.2f}", name])
    return summary
