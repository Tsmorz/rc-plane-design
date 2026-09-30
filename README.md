# RC Plane Design

## Summary
<img src="docs/three_view.png" width="90%" height="90%">

Aerodynamic design, mass/CG budget, and rib export for a sub-250 g, ESP32-S3-controlled, 3D-printed (Markforged Onyx rib + carbon spar + film) trainer. Built on [AeroSandbox](https://github.com/peterdsharpe/AeroSandbox) with NeuralFoil for low-Reynolds airfoil data.

**Wing airfoil: SD7062** (decided). `task airfoils` is kept for comparing against alternatives.

Everything is driven by `src/planedesign/config.py`: geometry, airfoils, and every component's mass and position.

## Setup
This project uses [uv](https://docs.astral.sh/uv/) and [go-task](https://taskfile.dev/).
1. install `uv` and `task`
2. `git clone https://github.com/Tsmorz/rc-plane-design.git`
3. `task init` to create the virtual environment and install the pre-commit hooks

## Running
Edit `config.py`, then:
```bash
task airfoils                    # candidate airfoils at your real Re
task analyze                     # mass budget, CG, static margin, trim, power
task analyze -- --solve-cg       # place the battery for the target static margin
task ribs                        # per-rib outlines + spar holes -> outputs/ribs/
task test                        # guardrails: mass limit, static margin, stall speed, straight spar
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
scripts/        airfoil comparison, design analysis, rib export
tests/          design guardrails
docs/           images used in this README
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

## Adding to the repo
1. When wanting to make new changes, please create a new branch first:\
`git checkout -b new-branch-name`

2. While making changes in your new branch, keep track of changes with:\
`git add file_that_changed`\
`git commit -m "a useful message"`\
`git push`

3. With new functionality, please create new unit and integration tests to make development easier in the future. Your tests should be added to the `tests` directory.
Tests should be run with: `task test`
