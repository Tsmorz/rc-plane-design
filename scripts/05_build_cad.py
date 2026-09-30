"""Generate printable ribs with CadQuery and export STEP/STL.

    python scripts/05_build_cad.py

Writes to outputs/cad/:
    rib_XX.step / .stl           one file per rib, flat as printed
    print_plate_NN.step / .stl   ribs packed onto plates sized to fit the printer bed
                                  (config.py: Design.printer); split across plates as needed
    wing_half_assembly.step      ribs at their span stations plus the square spar (import to Onshape)
    fuselage_pod.step / .stl     nose pod, lofted up to Design.pod_end_x (printed)
    aileron_rib_NN.step / .stl   aft piece of each rib inside the aileron span (also on the plates)
    htail.step / .stl            airfoil stabilizer (forward of the hinge gap)
    elevator.step / .stl         airfoil elevator (aft of the hinge gap)
    vtail.step / .stl            airfoil fin
    full_aircraft_assembly.step  both wing halves + pod + boom + tail, one file to eyeball the
                                  whole plane in a STEP viewer (e.g. FreeCAD, Onshape)

Aft of the pod, the fuselage is a stock carbon tube boom (not printed/exported) running to
the tail mount; see the printed boom length/OD below.

Requires the cad extra: `uv sync --extra cad`.
"""

from pathlib import Path

from planedesign import DESIGN
from planedesign.cad import boom_length_mm, build_control_ribs, build_ribs, export_all

OUT = Path(__file__).resolve().parents[1] / "outputs" / "cad"

if __name__ == "__main__":
    parts = build_ribs(DESIGN)
    print(f"{len(parts)} ribs per wing half")
    print(
        f"{len(build_control_ribs(DESIGN))} of them split at the hinge (aileron ribs)"
    )
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
    n_plates = sum(
        1
        for name in paths
        if name.startswith("print_plate_") and name.endswith(".step")
    )
    print(
        f"packed onto {n_plates} print plate(s) for the {DESIGN.printer.bed_width_mm:.0f} x "
        f"{DESIGN.printer.bed_depth_mm:.0f} x {DESIGN.printer.bed_height_mm:.0f} mm bed"
    )
    print(
        f"boom: {boom_length_mm(DESIGN):.0f} mm of {DESIGN.boom_od_mm:.0f} mm OD carbon "
        "tube, cut from stock (not printed)"
    )
    print(f"\n{len(paths)} files written to {OUT}")
