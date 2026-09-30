# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Aerodynamic design, mass/CG budget, and rib export for a sub-250 g, ESP32-S3-controlled, 3D-printed trainer RC plane. Built on AeroSandbox (AeroBuildup) with NeuralFoil for low-Reynolds airfoil data.

The package name is `planedesign` (in `src/planedesign/`). All design parameters live in `config.py`; everything else is derived from it.

## Commands

Uses uv + a Taskfile (go-task). Python 3.12. Runtime deps in `[project.dependencies]`; dev tooling (pytest, ruff, mypy, pre-commit) in `[dependency-groups.dev]`.

```bash
task init                 # uv sync + install pre-commit hooks
task airfoils             # compare candidate airfoils
task analyze              # mass budget, CG, static margin, trim, power
task analyze -- --solve-cg  # solve battery position for the target static margin
task ribs                 # export rib CSVs to outputs/ribs/
task format               # ruff format + ruff check --fix + mypy
task test                 # pytest with coverage over src/planedesign
task ci                   # format + test (local CI mirror)
task clean                # remove .venv, caches, build artifacts
```

Run a single test:
```bash
uv run pytest tests/design_test.py -v
```

## Layout

- `src/planedesign/` — the package.
  - `config.py` — design parameters and mass items (the file to edit).
  - `geometry.py` — config -> `asb.Airplane`.
  - `mass.py` — mass budget, CG, battery-position solver.
  - `analysis.py` — neutral point, static margin, trim, stall, power/flight time.
  - `airfoils.py`, `ribs.py` — NeuralFoil polars; rib outlines and CSV export.
- `scripts/` — numbered CLI entrypoints (`01_compare_airfoils.py`, `02_analyze_design.py`, `03_export_ribs.py`).
- `tests/` — design guardrails (mass limit, static margin, stall speed, straight spar). Test files end in `_test.py`.
- `docs/` — images used in the README.
- `outputs/` — generated plots and CSVs (git-ignored).
