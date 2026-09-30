"""Guardrails: run `pytest` after every config change."""

import numpy as onp
import pytest

from planedesign import DESIGN
from planedesign import analysis as A
from planedesign import mass as M
from planedesign.config import MAX_TAKEOFF_MASS_G, TARGET_TAKEOFF_MASS_G
from planedesign.ribs import rib_stations


def test_under_legal_limit():
    assert M.total_mass_g(DESIGN) < MAX_TAKEOFF_MASS_G


def test_under_design_target():
    assert M.total_mass_g(DESIGN) <= TARGET_TAKEOFF_MASS_G


def test_static_margin_trainer_range():
    assert 0.08 <= A.static_margin(DESIGN) <= 0.25


def test_battery_solver_hits_target_cg():
    target = 0.05
    d = M.with_battery_at(DESIGN, M.battery_x_for_cg(DESIGN, target))
    assert M.cg(d)[0] == pytest.approx(target, abs=1e-9)


def test_trainer_stall_speed():
    assert A.stall_speed(DESIGN) < 7.0


def test_spar_holes_are_collinear():
    """A straight carbon tube must pass through every rib's spar hole."""
    ribs = rib_stations(DESIGN)
    y = onp.array([r.y for r in ribs])
    for k in (0, 1):  # x and z of the spar center
        v = onp.array([r.spar_center[k] for r in ribs])
        residual = v - onp.polyval(onp.polyfit(y, v, 1), y)
        assert onp.max(onp.abs(residual)) < 1e-4  # 0.1 mm
