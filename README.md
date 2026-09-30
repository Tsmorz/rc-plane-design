# rc-plane-design

Aerodynamic design, mass/CG budget, and rib export for a sub-250 g, ESP32-S3-controlled,
3D-printed (Markforged Onyx rib + carbon spar + film) trainer. Built on
[AeroSandbox](https://github.com/peterdsharpe/AeroSandbox) with NeuralFoil for low-Reynolds airfoil data.

## Setup

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest                      # guardrails: mass limit, static margin, stall speed, straight spar
```

## Workflow

Everything is driven by **`src/planedesign/config.py`**: geometry, airfoils, and every component's
mass and position. Change it, then:

```bash
python scripts/01_compare_airfoils.py              # candidate airfoils at your real Re
python scripts/02_analyze_design.py                # mass budget, CG, static margin, trim, power
python scripts/02_analyze_design.py --solve-cg     # place the battery for the target static margin
python scripts/03_export_ribs.py                   # per-rib outlines + spar holes -> outputs/ribs/
pytest
```

Outputs (plots, rib CSVs) land in `outputs/` (git-ignored).

## Layout

```
src/planedesign/
  config.py     design parameters + mass items (edit this)
  geometry.py   config -> asb.Airplane (wing, tail, fin, pod/boom)
  mass.py       mass budget, CG, battery-position solver, what-if helpers
  analysis.py   neutral point, static margin, trim via asb.Opti, stall, power/flight time
  airfoils.py   NeuralFoil polars and summaries
  ribs.py       rib stations in each rib's flat 2D frame, CSV export
scripts/        01-03 as above
tests/          design guardrails
```

## Reading the results

- **Static margin**: aim for 12-20 % MAC on a trainer. Below 5 % is twitchy or unstable; above 25 % is
  nose-heavy. Rather than adding ballast, fix it with `--solve-cg` (moves the battery).
- **Tail incidence**: `trim()` solves stab incidence and alpha for L = W and Cm = 0. Build the stab at
  the value for your cruise speed; the spread across speeds is what the elevator must supply.
- **Flight time**: "ideal" is steady level flight on a perfect surface. "Real" applies
  `real_world_power_factor` (default 2.5x). Trust the real column, then calibrate the factor from
  your first flight logs.
- **NeuralFoil confidence** below ~0.9 means that polar is extrapolated; don't choose an airfoil on it.

## Caveats

AeroBuildup is a fast, differentiable buildup method, not CFD. It is good for trends, sizing, CG and
trim, but it does not capture film scalloping between ribs, laminar-bubble details, or propwash. Verify
with ballasted glide tests before the first powered flight, and treat logged flight data as the truth.

## Next steps

- CadQuery rib generator: read `outputs/ribs/*.csv`, add lightening holes, LE-rod notch, spar hole
  clearance, and jig tabs from `twist_deg`, then export STEP for Eiger.
- Feed measured part masses back into `config.py` as you build (use a 0.1 g scale).
