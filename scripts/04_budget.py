"""Print the mass and power budget and write docs/budget.md.

    python scripts/04_budget.py

Edit src/planedesign/config.py: set `measured_g` on a Component once it is weighed,
and update `power_items` as you measure real current draw. Commit docs/budget.md to
keep a history of how the budget changes as the build progresses.
"""

from pathlib import Path

from planedesign import DESIGN
from planedesign.budget import mass_markdown, power_markdown, write_markdown

DOC = Path(__file__).resolve().parents[1] / "docs" / "budget.md"

if __name__ == "__main__":
    print("=== MASS ===")
    print(mass_markdown(DESIGN))
    print("\n=== POWER ===")
    print(power_markdown(DESIGN))
    print(f"\nwritten: {write_markdown(DESIGN, DOC)}")
