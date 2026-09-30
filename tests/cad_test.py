"""CAD guardrails; skipped when the optional `cad` extra is not installed."""

import pytest

pytest.importorskip("cadquery")

import cadquery as cq

from planedesign import DESIGN
from planedesign.cad import build_ribs, export_all


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
        "print_plate.stl",
        "wing_half_assembly.step",
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
    n = len(parts)
    plate = cq.importers.importStep(str(exported["print_plate.step"])).val()
    assert len(plate.Solids()) == n
    asm = cq.importers.importStep(str(exported["wing_half_assembly.step"])).val()
    assert len(asm.Solids()) == n + 1  # ribs + spar tube
