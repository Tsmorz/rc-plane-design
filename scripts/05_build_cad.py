"""Generate printable ribs with CadQuery and export STEP/STL.

    python scripts/05_build_cad.py

Writes to outputs/cad/:
    rib_XX.step / .stl        one file per rib, flat as printed
    print_plate.step / .stl   every rib on one plate (drag the STEP into Eiger)
    wing_half_assembly.step   ribs at their span stations plus the square spar (import to Onshape)

Requires the cad extra: `uv sync --extra cad`.
"""

from pathlib import Path

from planedesign import DESIGN
from planedesign.cad import build_ribs, export_all

OUT = Path(__file__).resolve().parents[1] / "outputs" / "cad"

if __name__ == "__main__":
    parts = build_ribs(DESIGN)
    print(f"{len(parts)} ribs per wing half")
    total_mm3 = 0.0
    for r, w in parts:
        bb = w.val().BoundingBox()
        vol = w.val().Volume()
        total_mm3 += vol
        print(
            f"  rib {r.index:02d}  {bb.xlen:6.1f} x {bb.ylen:5.1f} mm  "
            f"volume {vol:7.1f} mm^3  {vol / 1000 * DESIGN.rib.density_g_cc:.2f} g"
        )
    half_g = total_mm3 / 1000 * DESIGN.rib.density_g_cc
    print(
        f"ribs, one wing half: {half_g:.1f} g (about {2 * half_g:.0f} g for the full wing)"
    )
    paths = export_all(DESIGN, OUT)
    print(f"\n{len(paths)} files written to {OUT}")
