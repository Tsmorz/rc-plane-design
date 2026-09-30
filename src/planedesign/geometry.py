"""Turn the Design config into AeroSandbox geometry."""

from __future__ import annotations

import aerosandbox as asb
import aerosandbox.numpy as np

from .config import Design, FinSpec, SurfaceSpec


def surface_xsecs(s: SurfaceSpec, incidence_offset=0.0) -> list[asb.WingXSec]:
    """Cross-sections for a tapered surface with constant quarter-chord sweep.

    A section is added at each end of the control surface (if any), which AeroSandbox
    needs to know where the flap starts and stops; the planform is unchanged.
    `incidence_offset` may be an Opti variable (used to trim the tail).
    """
    af = asb.Airfoil(s.airfoil)
    semi = s.span / 2
    cs = s.control_surface
    etas = sorted({0.0, 1.0, *((cs.span_start, cs.span_end) if cs else ())})
    xsecs = []
    for eta in etas:
        chord = s.root_chord + eta * (s.tip_chord - s.root_chord)
        y = eta * semi
        # keep the quarter-chord line at the requested sweep
        x_qc = s.x_le_root + s.root_chord / 4 + y * np.tand(s.quarter_chord_sweep_deg)
        xsecs.append(
            asb.WingXSec(
                xyz_le=[x_qc - chord / 4, y, s.z_root + y * np.tand(s.dihedral_deg)],
                chord=chord,
                twist=s.root_incidence_deg - eta * s.washout_deg + incidence_offset,
                airfoil=af,
                # the flap runs from this section to the next one outboard
                control_surfaces=[
                    asb.ControlSurface(
                        name=cs.name,
                        symmetric=cs.symmetric,
                        hinge_point=1 - cs.chord_fraction,
                    )
                ]
                if cs and eta == cs.span_start
                else [],
            )
        )
    return xsecs


def make_wing(s: SurfaceSpec, incidence_offset=0.0) -> asb.Wing:
    return asb.Wing(
        name=s.name, symmetric=True, xsecs=surface_xsecs(s, incidence_offset)
    )


def make_fin(f: FinSpec) -> asb.Wing:
    af = asb.Airfoil(f.airfoil)
    return asb.Wing(
        name="Vertical Stabilizer",
        symmetric=False,
        xsecs=[
            asb.WingXSec(
                xyz_le=[f.x_le_root, 0, f.z_root], chord=f.root_chord, airfoil=af
            ),
            asb.WingXSec(
                xyz_le=[
                    f.x_le_root + f.height * np.tand(f.le_sweep_deg),
                    0,
                    f.z_root + f.height,
                ],
                chord=f.tip_chord,
                airfoil=af,
            ),
        ],
    )


def make_fuselage(d: Design) -> asb.Fuselage:
    return asb.Fuselage(
        name="Fuselage",
        xsecs=[
            asb.FuselageXSec(xyz_c=[x, 0, 0], radius=r) for x, r in d.fuselage_stations
        ],
    )


def make_airplane(
    d: Design, xyz_ref=(0.0, 0.0, 0.0), tail_incidence_deg=0.0
) -> asb.Airplane:
    """Full airplane. `tail_incidence_deg` is added to the htail incidence (trim variable)."""
    return asb.Airplane(
        name="ESP32-S3 sub-250g trainer",
        xyz_ref=list(xyz_ref),
        wings=[
            make_wing(d.wing),
            make_wing(d.htail, incidence_offset=tail_incidence_deg),
            make_fin(d.vtail),
        ],
        fuselages=[make_fuselage(d)] if d.fuselage_stations else [],
    )
