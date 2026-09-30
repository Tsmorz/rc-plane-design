"""CAD guardrails; skipped when the optional `cad` extra is not installed."""

import pytest

pytest.importorskip("cadquery")

import cadquery as cq

from planedesign import DESIGN
from planedesign.cad import (
    build_control_ribs,
    build_fin,
    build_htail,
    build_ribs,
    build_tail_plate,
    export_all,
    hinge_x_mm,
)


@pytest.fixture(scope="module")
def parts():
    return build_ribs(DESIGN)


@pytest.fixture(scope="module")
def exported(parts, tmp_path_factory):
    return export_all(DESIGN, tmp_path_factory.mktemp("cad"), parts)


def test_ribs_are_valid_single_solids(parts):
    assert len(parts) > 1
    for _, w in parts:
        assert w.val().isValid()
        assert len(w.solids().vals()) == 1


def test_export_writes_step_and_stl(exported):
    for name in (
        "rib_00.step",
        "rib_00.stl",
        "print_plate_01.stl",
        "wing_half_assembly.step",
        "fuselage_pod.step",
        "htail.step",
        "elevator.step",
        "aileron_rib_05.step",
        "vtail.step",
        "full_aircraft_assembly.step",
    ):
        assert exported[name].stat().st_size > 0


def test_step_round_trips_as_one_closed_solid(parts, exported):
    """A rib must re-import from STEP as one solid with every face intact.

    Offset curves left in the geometry used to be dropped on import, leaving loose
    shells (unconnected pieces) in Onshape and Eiger.
    """
    for r, w in parts:
        back = cq.importers.importStep(str(exported[f"rib_{r.index:02d}.step"])).val()
        assert len(back.Solids()) == 1, f"rib {r.index} lost its solid"
        assert len(back.Shells()) == 1
        assert len(back.Faces()) == len(w.val().Faces())


def test_plate_and_assembly_keep_every_part(parts, exported):
    n = len(parts) + len(build_control_ribs(DESIGN))
    plate_names = sorted(
        name
        for name in exported
        if name.startswith("print_plate_") and name.endswith(".step")
    )
    n_on_plates = sum(
        len(cq.importers.importStep(str(exported[name])).val().Solids())
        for name in plate_names
    )
    assert n_on_plates == n
    asm = cq.importers.importStep(str(exported["wing_half_assembly.step"])).val()
    assert len(asm.Solids()) == n + 1  # ribs + aileron ribs + spar tube


def test_plates_fit_the_printer_bed(exported):
    printer = DESIGN.printer
    plate_names = sorted(
        name
        for name in exported
        if name.startswith("print_plate_") and name.endswith(".step")
    )
    assert plate_names
    for name in plate_names:
        bb = cq.importers.importStep(str(exported[name])).val().BoundingBox()
        assert bb.xlen <= printer.bed_width_mm
        assert bb.ylen <= printer.bed_depth_mm
        assert bb.zlen <= printer.bed_height_mm


def test_split_ribs_leave_the_hinge_gap():
    """Fixed and aileron pieces of a split rib must not touch or overlap."""
    cs = DESIGN.wing.control_surface
    control = dict((r.index, w) for r, w in build_control_ribs(DESIGN))
    assert control, "expected ribs inside the aileron span"
    for r, fixed in build_ribs(DESIGN):
        if r.index not in control:
            continue
        hinge = hinge_x_mm(r.chord, cs)
        gap = control[r.index].val().BoundingBox().xmin - fixed.val().BoundingBox().xmax
        assert gap == pytest.approx(cs.hinge_gap_mm, abs=1e-3)
        assert (
            fixed.val().BoundingBox().xmax
            < hinge
            < control[r.index].val().BoundingBox().xmin
        )
        assert control[r.index].val().isValid()
        assert len(control[r.index].solids().vals()) == 1


def test_elevator_is_separate_from_the_stab_by_the_hinge_gap():
    stab, elevator = build_htail(DESIGN.htail, DESIGN.printer.tail_te_mm)
    assert stab.val().isValid() and elevator.val().isValid()
    assert len(stab.solids().vals()) == len(elevator.solids().vals()) == 1
    assert stab.val().intersect(elevator.val()).Volume() == pytest.approx(0, abs=1e-6)
    # together they are the full airfoil loft minus the hinge strip (OCC volumes of ruled
    # lofts are only good to about a percent, so compare loosely)
    full = build_tail_plate(DESIGN.htail, DESIGN.printer.tail_te_mm).val().Volume()
    both = stab.val().Volume() + elevator.val().Volume()
    assert both == pytest.approx(full, rel=0.03)


def test_tail_surfaces_are_airfoils_not_plates():
    """Thickness must follow the airfoil: max ~6 % chord for NACA 0006, not a flat slab."""
    te = DESIGN.printer.tail_te_mm
    bb = build_tail_plate(DESIGN.htail, te).val().BoundingBox()
    assert bb.zlen == pytest.approx(0.06 * DESIGN.htail.root_chord * 1000, rel=0.1)
    bb = build_fin(DESIGN.vtail, te).val().BoundingBox()
    assert bb.ylen == pytest.approx(0.06 * DESIGN.vtail.root_chord * 1000, rel=0.1)
    assert bb.zlen == pytest.approx(DESIGN.vtail.height * 1000, rel=1e-3)
