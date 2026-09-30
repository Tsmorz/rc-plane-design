"""Export rib stations (outline + spar hole + twist) for one wing half.

    python scripts/03_export_ribs.py

Writes outputs/ribs/rib_XX.csv (outline in mm) and rib_summary.csv. These feed the
CadQuery rib generator (next step), which adds lightening holes, LE-rod notch,
spar hole clearance, and jig tabs from twist_deg.
"""

from pathlib import Path

from planedesign import DESIGN
from planedesign.ribs import export_csv, rib_stations

OUT = Path(__file__).resolve().parents[1] / "outputs" / "ribs"

if __name__ == "__main__":
    ribs = rib_stations(DESIGN)
    path = export_csv(ribs, OUT)
    print(
        f"{len(ribs)} ribs per wing half ({2 * len(ribs) - 1} total incl. shared root rib)"
    )
    for r in ribs:
        print(
            f"  rib {r.index:02d}  y={r.y * 1000:5.0f} mm  chord={r.chord * 1000:5.1f} mm  "
            f"twist={r.twist_deg:+.2f} deg  spar @ ({r.spar_center[0] * 1000:.1f}, {r.spar_center[1] * 1000:.1f}) mm"
        )
    print(f"\nsummary: {path}")
