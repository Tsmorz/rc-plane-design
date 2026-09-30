"""Guardrails: run `pytest` after every config change."""

from dataclasses import replace

import numpy as onp
import pytest

from planedesign import DESIGN, budget
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


def test_peak_current_within_battery_limit():
    """Worst-case simultaneous draw must not exceed the pack's C rating."""
    s = budget.power_summary(DESIGN)
    assert s.peak_current_a <= s.max_current_a


def test_measured_mass_overrides_estimate():
    c = replace(DESIGN.components[0], measured_g=DESIGN.components[0].mass_g + 5)
    d = replace(DESIGN, components=[c, *DESIGN.components[1:]])
    assert abs(M.total_mass_g(d) - M.total_mass_g(DESIGN) - 5) < 1e-9


def test_control_surfaces_are_modeled():
    """Ailerons and elevator must reach AeroSandbox, or trim/authority numbers are fiction."""
    from planedesign.geometry import make_airplane

    names = {
        cs.name
        for w in make_airplane(DESIGN).wings
        for x in w.xsecs
        for cs in x.control_surfaces
    }
    assert names == {"aileron", "elevator"}


def test_elevator_can_trim_the_speed_range():
    """Stab fixed, elevator alone must trim from near-stall to fast cruise with margin."""
    cs = DESIGN.htail.control_surface
    for v in (1.3 * A.stall_speed(DESIGN), 10.0, 16.0):
        assert A.trim_elevator(DESIGN, v).authority_used < 0.6, f"{v:.1f} m/s"
    assert cs.max_deflection_deg > 0


def test_ailerons_give_a_usable_roll_rate():
    """Trainer: enough to bank briskly (>=60 deg/s) without being a snap-roller (<=360)."""
    rate = A.roll_rate_dps(DESIGN, 10.0)
    assert 60 <= rate <= 360


def test_aileron_ends_sit_on_rib_stations():
    """Aileron span ends must land on ribs, which act as closeouts."""
    cs = DESIGN.wing.control_surface
    ys = [r.y for r in rib_stations(DESIGN)]
    semi = DESIGN.wing.span / 2
    for eta in (cs.span_start, cs.span_end):
        assert min(abs(y - eta * semi) for y in ys) < 1e-6
