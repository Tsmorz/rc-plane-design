"""CadQuery rib generator: printable ribs, a print plate, and a wing-half assembly.

Each rib is built flat in the XY plane (chord along +x, leading edge at the origin,
extruded +z) so it prints lying on the bed. Features added on top of the bare airfoil:

- trailing edge cut back to `te_min_mm` so it is printable
- square spar hole on the camber line (collinear across ribs, so a straight tube fits)
- solid cap-to-cap web band through the spar, plus a square collar on one face
- open slot through the nose for the LE carbon rod (snaps in from the front)
- Warren-truss lightening (diagonal members inside a solid rim) fore and aft of the spar
- ribs inside the aileron span are split at the hinge line into a fixed rib and a separate
  aileron rib (own LE rod notch, truss), with `hinge_gap_mm` between them for the film hinge

The square spar is the jig. Each rib's square hole is rotated by that rib's `twist_deg`,
so once the ribs are slid onto the straight tube their incidence and washout are set and
they cannot rotate about the spar. Only the spanwise position is left to set, using marks
or spacers at the rib pitch.

Export is STEP (Onshape) and STL (Eiger). All units are mm.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as onp

from .config import (
    ControlSurfaceSpec,
    Design,
    FinSpec,
    PrinterSpec,
    RibSpec,
    SurfaceSpec,
)
from .ribs import Rib, rib_stations


def _cq():
    try:
        import cadquery as cq
    except ImportError as e:  # pragma: no cover
        raise ImportError("CadQuery missing: run `uv sync --extra cad`") from e
    return cq


def _surfaces(outline_mm: onp.ndarray) -> tuple[onp.ndarray, onp.ndarray]:
    """Split Selig-ordered coordinates into upper and lower curves, LE -> TE."""
    i_le = int(onp.argmin(outline_mm[:, 0]))
    upper = outline_mm[: i_le + 1][::-1]
    lower = outline_mm[i_le:]
    return upper, lower


def _z(curve: onp.ndarray, x):
    return onp.interp(x, curve[:, 0], curve[:, 1])


def _to_nurbs(wp):
    """Replace offset curves/surfaces with plain B-splines.

    `offset2D` leaves OCC offset curves on the edges. Booleans against them produce
    faces that STEP writes but does not read back (they arrive as loose shells, i.e.
    unconnected pieces). Plain B-splines round-trip cleanly.
    """
    from OCP.BRepBuilderAPI import BRepBuilderAPI_NurbsConvert

    cq = _cq()
    shape = BRepBuilderAPI_NurbsConvert(wp.val().wrapped, True).Shape()
    return cq.Workplane(obj=cq.Shape.cast(shape))


def _outline_wp(cq, up: onp.ndarray, lo: onp.ndarray, plane="XY", origin=(0, 0, 0)):
    """Build the closed rib outline: upper spline, TE cut, lower spline back to the LE."""
    w = cq.Workplane(plane, origin=origin).moveTo(*up[0])
    w = w.spline([tuple(p) for p in up[1:]], includeCurrent=True)
    w = w.lineTo(*lo[-1])
    return w.spline([tuple(p) for p in lo[-2::-1]], includeCurrent=True).close()


def _shrunk_triangle(cq, tri, inset, spec: RibSpec):
    """Triangle shrunk by half a member width and clipped to the inset, or None."""
    try:
        p = (
            cq.Workplane("XY")
            .polyline(tri)
            .close()
            .offset2D(-spec.truss_member_mm / 2)
            .extrude(spec.thickness_mm)
            .intersect(inset)
        )
    except ValueError:  # degenerate sliver: nothing left to cut
        return None
    if not p.vals() or p.val().Volume() / spec.thickness_mm < spec.min_hole_mm**2 / 2:
        return None
    return p


def _truss_pockets(cq, upper, lower, inset, x0: float, x1: float, spec: RibSpec):
    """Warren-truss pockets between x0 and x1, clipped to the inset outline.

    The span is split into bays. Each bay is cut by one diagonal (alternating
    direction) into two triangles, and every triangle is shrunk by half the member
    width so the material left between neighbours is `truss_member_mm` wide.
    """
    xs_probe = onp.linspace(x0, x1, 50)
    t_max = float((_z(upper, xs_probe) - _z(lower, xs_probe)).max())
    n = max(1, round((x1 - x0) / (spec.truss_bay_ratio * t_max)))
    xs = onp.linspace(x0, x1, n + 1)
    m = 2 * spec.truss_member_mm
    pockets = []
    for i in range(n):
        xa, xb = float(xs[i]), float(xs[i + 1])
        seg = onp.linspace(xa, xb, 10)
        ylo = float(_z(lower, seg).min()) - m
        yhi = float(_z(upper, seg).max()) + m
        if i % 2 == 0:
            tris = [
                [(xa, ylo), (xb, yhi), (xb, ylo)],
                [(xa, ylo), (xa, yhi), (xb, yhi)],
            ]
        else:
            tris = [
                [(xa, yhi), (xb, ylo), (xb, yhi)],
                [(xa, yhi), (xa, ylo), (xb, ylo)],
            ]
        for tri in tris:
            p = _shrunk_triangle(cq, tri, inset, spec)
            if p is not None:
                pockets.append(p)
    return pockets


def hinge_x_mm(chord_m: float, cs: ControlSurfaceSpec) -> float:
    """Hinge line x (mm, rib frame) for a rib of this chord."""
    return (1 - cs.chord_fraction) * chord_m * 1000


def splits_at_hinge(d: Design, r: Rib) -> bool:
    """Return True for ribs strictly inside the wing's control-surface span.

    The two ribs at the span ends stay whole: they close out the fixed wing and the
    control surface bay, and the film is slit beside them.
    """
    cs = d.wing.control_surface
    if cs is None:
        return False
    semi = d.wing.span / 2
    eps = 1e-6
    return cs.span_start * semi + eps < r.y < cs.span_end * semi - eps


def _te_cut(
    upper: onp.ndarray, lower: onp.ndarray, c_mm: float, spec: RibSpec
) -> float:
    """X where the airfoil is still `te_min_mm` thick: the printable trailing edge."""
    xs = onp.linspace(0.5 * c_mm, c_mm, 400)
    thick = _z(upper, xs) - _z(lower, xs)
    ok = onp.where(thick >= spec.te_min_mm)[0]
    return float(xs[ok[-1]]) if len(ok) else c_mm


def _clip(upper, lower, x0: float, x1: float):
    """Upper and lower curves restricted to x0 <= x <= x1 (ends interpolated)."""
    out = []
    for c in (upper, lower):
        mid = c[(c[:, 0] > x0) & (c[:, 0] < x1)]
        out.append(onp.vstack([[x0, _z(c, x0)], mid, [x1, _z(c, x1)]]))
    return out[0], out[1]


def _le_rod_slot(cq, spec: RibSpec, upper, lower, x0: float, t_rib: float):
    """Rod-diameter channel opening forward from x0, plus the cutter's rod x and radius."""
    rod_r = spec.le_rod_mm / 2 + 0.1
    rod_x = x0 + spec.le_rod_mm / 2 + 0.2
    rod_z = 0.5 * (_z(upper, rod_x) + _z(lower, rod_x))
    cutter = (
        cq.Workplane("XY")
        .center(rod_x, rod_z)
        .circle(rod_r)
        .extrude(t_rib)
        .union(
            cq.Workplane("XY")
            .center(x0 - 1.0, rod_z)
            .rect(2 * (rod_x - x0 + 1.0), 2 * rod_r)
            .extrude(t_rib)
        )
    )
    return cutter, rod_x, rod_r


def build_control_rib(r: Rib, spec: RibSpec, cs: ControlSurfaceSpec):
    """Build the aft piece of a split rib: the control surface's own rib (flat, extruded +z).

    Runs from just behind the hinge gap to the trailing edge, with a notch at its nose
    for a carbon rod (the control surface's leading-edge stiffener) and truss pockets.
    """
    cq = _cq()
    c_mm = r.chord * 1000
    upper, lower = _surfaces(r.outline * 1000)
    t_rib = spec.thickness_mm
    x0 = hinge_x_mm(r.chord, cs) + cs.hinge_gap_mm / 2
    x_cut = _te_cut(upper, lower, c_mm, spec)
    up, lo = _clip(upper, lower, x0, x_cut)

    rib = _outline_wp(cq, up, lo).extrude(t_rib)
    inset = _to_nurbs(_outline_wp(cq, up, lo).offset2D(-spec.min_web_mm).extrude(t_rib))
    slot, rod_x, rod_r = _le_rod_slot(cq, spec, upper, lower, x0, t_rib)
    x_front = rod_x + rod_r + spec.min_web_mm
    x_back = x_cut - spec.min_web_mm
    if x_back > x_front:
        for p in _truss_pockets(cq, upper, lower, inset, x_front, x_back, spec):
            rib = rib.cut(p)
    return rib.cut(slot)


def build_rib(r: Rib, spec: RibSpec, x_stop: float | None = None):
    """One rib as a CadQuery Workplane (flat, extruded +z).

    `x_stop` ends the rib early (at the hinge line of a split rib); the aft piece is
    built by `build_control_rib`.
    """
    cq = _cq()
    c_mm = r.chord * 1000
    upper, lower = _surfaces(r.outline * 1000)
    t_rib = spec.thickness_mm

    # --- trim the trailing edge back to te_min ---------------------------
    x_cut = _te_cut(upper, lower, c_mm, spec)
    if x_stop is not None:
        x_cut = min(x_cut, x_stop)
    up = upper[upper[:, 0] < x_cut]
    lo = lower[lower[:, 0] < x_cut]
    up = onp.vstack([up, [x_cut, _z(upper, x_cut)]])
    lo = onp.vstack([lo, [x_cut, _z(lower, x_cut)]])

    rib = _outline_wp(cq, up, lo).extrude(t_rib)
    inset = _to_nurbs(_outline_wp(cq, up, lo).offset2D(-spec.min_web_mm).extrude(t_rib))

    sx, sz = r.spar_center[0] * 1000, r.spar_center[1] * 1000
    hole_w = spec.spar_width_mm + spec.spar_clearance_mm
    th = math.radians(r.twist_deg)

    def square(w: float):
        # Square sides are level in the world, which is the rib frame rotated by +twist.
        corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        return [
            (
                sx + (cx * math.cos(th) - cz * math.sin(th)) * w / 2,
                sz + (cx * math.sin(th) + cz * math.cos(th)) * w / 2,
            )
            for cx, cz in corners
        ]

    hole_pts = square(hole_w)
    slot, rod_x, rod_r = _le_rod_slot(cq, spec, upper, lower, 0.0, t_rib)

    # --- truss lightening: nose bay ahead of the spar, then aft to the TE ---
    # A solid band runs cap to cap through the spar, so spar bending and shear load
    # the full rib depth instead of a thin ring hanging off narrow truss posts.
    # Pockets are shrunk by half a member, so stop them that much short of the band.
    band = max(p[0] - sx for p in hole_pts) + spec.spar_web_mm
    gap = max(band - spec.truss_member_mm / 2, 0.0)
    x_front = rod_x + rod_r + spec.min_web_mm
    x_back = x_cut - spec.min_web_mm
    for x0, x1 in ((x_front, sx - gap), (sx + gap, x_back)):
        for p in _truss_pockets(cq, upper, lower, inset, x0, x1, spec):
            rib = rib.cut(p)

    # --- spar collar: sleeve on the top face, so the tube is held over more than
    # one rib thickness (glue area, and the twist is set by a longer square) ----
    if spec.spar_collar_mm > 0:
        collar = (
            cq.Workplane("XY")
            .workplane(offset=t_rib)
            .polyline(square(hole_w + 2 * spec.spar_collar_wall_mm))
            .close()
            .extrude(spec.spar_collar_mm)
        )
        rib = rib.union(collar)

    # --- square spar hole, rotated to the rib's twist -----------------------
    rib = rib.cut(
        cq.Workplane("XY")
        .polyline(hole_pts)
        .close()
        .extrude(t_rib + spec.spar_collar_mm)
    )

    # --- LE rod slot: rod-diameter channel from in front of the nose inward --
    rib = rib.cut(slot)
    return rib


def build_ribs(d: Design):
    """[(Rib, Workplane)] for one wing half. Ribs inside the aileron span stop at the hinge."""
    cs = d.wing.control_surface
    return [
        (
            r,
            build_rib(
                r,
                d.rib,
                x_stop=hinge_x_mm(r.chord, cs) - cs.hinge_gap_mm / 2
                if cs and splits_at_hinge(d, r)
                else None,
            ),
        )
        for r in rib_stations(d)
    ]


def build_control_ribs(d: Design):
    """[(Rib, Workplane)] aileron ribs for one wing half (empty without an aileron)."""
    cs = d.wing.control_surface
    if cs is None:
        return []
    return [
        (r, build_control_rib(r, d.rib, cs))
        for r in rib_stations(d)
        if splits_at_hinge(d, r)
    ]


def pack_plates(parts, printer: PrinterSpec):
    """Shelf-pack parts into one or more plates that fit the printer's bed footprint.

    Parts are placed left-to-right (bed width) in rows (shelves); a row wraps to a new
    shelf when it runs out of width, and a shelf that would exceed the bed depth starts
    a new plate instead. Height is checked against the bed but never packed (every part
    here is a thin, flat print). Raises if a single part's own footprint or height
    can't fit the bed at all, since no amount of packing fixes that.
    """
    gap = printer.plate_gap_mm
    bw, bd, bh = printer.bed_width_mm, printer.bed_depth_mm, printer.bed_height_mm

    for name, w in parts:
        bb = w.val().BoundingBox()
        if bb.zlen > bh:
            raise ValueError(
                f"{name} is {bb.zlen:.1f} mm tall, taller than the {bh:.1f} mm bed height"
            )
        if bb.xlen > bw or bb.ylen > bd:
            raise ValueError(
                f"{name} footprint {bb.xlen:.1f} x {bb.ylen:.1f} mm does not fit the "
                f"{bw:.1f} x {bd:.1f} mm bed in any orientation"
            )

    plates: list[list] = [[]]
    x_cursor = y_cursor = row_h = 0.0
    for name, w in parts:
        bb = w.val().BoundingBox()
        dx, dy = bb.xlen, bb.ylen
        if x_cursor > 0 and x_cursor + dx > bw:  # wrap to a new row
            x_cursor = 0.0
            y_cursor += row_h + gap
            row_h = 0.0
        if y_cursor > 0 and y_cursor + dy > bd:  # this plate is full: start another
            plates.append([])
            x_cursor = y_cursor = row_h = 0.0
        moved = w.translate((x_cursor - bb.xmin, y_cursor - bb.ymin, 0))
        plates[-1].append((name, moved.val()))
        x_cursor += dx + gap
        row_h = max(row_h, dy)

    cq = _cq()
    return [cq.Workplane("XY").newObject([s for _, s in plate]) for plate in plates]


def _stand_up(w, r: Rib):
    """Flat rib -> world position at its span station, twisted about its spar axis."""
    sx, sz = r.spar_center[0] * 1000, r.spar_center[1] * 1000
    # flat (x, z_airfoil, thickness) -> world (x, thickness, z_airfoil); mirrored
    # first so the spar collar points outboard (clear of the root joint)
    s = w.mirror("XY").rotate((0, 0, 0), (1, 0, 0), 90).translate((0, r.y * 1000, 0))
    return s.rotate((sx, 0, sz), (sx, 1, sz), r.twist_deg)


def wing_assembly(d: Design, parts, control_parts=None):
    """Ribs stood up at their span stations, twisted about the spar, plus the spar.

    `control_parts` are the aileron ribs (default: built from `d`); they sit at the same
    stations as the fixed ribs they were split from.
    """
    cq = _cq()
    t_mm = d.rib.thickness_mm
    asm = cq.Assembly(name="wing_half")
    centers = []
    for r, w in parts:
        sx, sz = r.spar_center[0] * 1000, r.spar_center[1] * 1000
        asm.add(_stand_up(w, r), name=f"rib_{r.index:02d}")
        centers.append((sx, r.y * 1000 + t_mm / 2, sz))
    if control_parts is None:
        control_parts = build_control_ribs(d)
    for r, w in control_parts:
        asm.add(_stand_up(w, r), name=f"aileron_rib_{r.index:02d}")
    (x0, y0, z0), (x1, y1, z1) = centers[0], centers[-1]
    length = math.dist(centers[0], centers[-1])
    tube = (
        cq.Workplane("XY")
        .rect(d.rib.spar_width_mm, d.rib.spar_width_mm)
        .extrude(length)
    )
    # tube axis +z -> along the line between the first and last spar centers
    v = onp.array([x1 - x0, y1 - y0, z1 - z0]) / length
    axis = onp.cross([0, 0, 1], v)
    ang = math.degrees(math.acos(float(v[2])))
    if onp.linalg.norm(axis) > 1e-9:
        tube = tube.rotate((0, 0, 0), tuple(axis), ang)
    asm.add(tube.translate((x0, y0 - 0, z0)), name="spar_tube")
    return asm


def full_aircraft_assembly(d: Design, parts, control_parts=None):
    """Both wing halves, pod, boom, and tail surfaces, all in world (x aft, y right, z up).

    `wing_assembly` already places its ribs/spar in world coordinates for the right
    half, so the left half is just its mirror. Pod, boom, and tail plates are built in
    their own local frames and translated to their config x/z positions.
    """
    cq = _cq()
    asm = cq.Assembly(name="aircraft")
    right = wing_assembly(d, parts, control_parts)
    for sub in right.children:
        asm.add(sub.obj, name=f"right_{sub.name}", loc=sub.loc)
        # mirror across the XZ plane (normal = y) to get the left wing half
        asm.add(sub.obj.mirror("XZ"), name=f"left_{sub.name}", loc=sub.loc)

    asm.add(build_pod(d), name="fuselage_pod")

    boom_len = boom_length_mm(d)
    boom = (
        cq.Workplane("YZ")
        .workplane(offset=d.pod_end_x * 1000)
        .circle(d.boom_od_mm / 2)
        .extrude(boom_len)
    )
    asm.add(boom, name="boom")

    htail_loc = cq.Location(
        cq.Vector(d.htail.x_le_root * 1000, 0, d.htail.z_root * 1000)
    )
    stab, elevator = build_htail(d.htail, d.printer.tail_te_mm)
    asm.add(stab, name="htail", loc=htail_loc)
    if elevator is not None:
        asm.add(elevator, name="elevator", loc=htail_loc)

    vtail = build_fin(d.vtail, d.printer.tail_te_mm)
    asm.add(
        vtail,
        name="vtail",
        loc=cq.Location(cq.Vector(d.vtail.x_le_root * 1000, 0, d.vtail.z_root * 1000)),
    )
    return asm


def build_pod(d: Design):
    """Nose pod as a loft through the `fuselage_stations` up to `pod_end_x`.

    Aft of `pod_end_x` the fuselage is a constant-diameter carbon tube boom (stock,
    not printed) rather than more loft, so only the nose is built/exported here; see
    `boom_length_mm`.
    """
    cq = _cq()
    stations = [s for s in d.fuselage_stations if s[0] <= d.pod_end_x]
    if len(stations) < 2:
        raise ValueError("need at least 2 fuselage_stations at or before pod_end_x")
    wp = cq.Workplane("YZ")
    sections = [
        wp.workplane(offset=x * 1000).circle(max(r * 1000, 1e-3)) for x, r in stations
    ]
    body = sections[0]
    for s in sections[1:]:
        body = body.add(s)
    return body.loft(ruled=True)


def boom_length_mm(d: Design) -> float:
    """Length of carbon-tube boom needed from the pod's aft end to the last fuselage station."""
    xs = [x for x, _ in d.fuselage_stations]
    return (max(xs) - d.pod_end_x) * 1000


def _airfoil_curves(airfoil: str, chord_mm: float, te_mm: float):
    """Upper and lower curves (LE -> TE, mm) of a tail airfoil with a printable trailing edge.

    Thickness is added linearly from the nose to the TE, so the chord (and so the hinge
    position) is unchanged and only the aft part gets slightly fatter.
    """
    import aerosandbox as asb

    coords = asb.Airfoil(airfoil).coordinates * chord_mm
    upper, lower = _surfaces(coords)
    blend = 0.5 * te_mm / chord_mm
    up = upper + onp.column_stack([0 * upper[:, 0], blend * upper[:, 0]])
    lo = lower + onp.column_stack([0 * lower[:, 0], -blend * lower[:, 0]])
    # drop the original TE points' thickness: the ends are now exactly +-te/2
    return up, lo


def _loft_sections(cq, sections):
    """Ruled loft through [(up, lo, plane, origin, x_offset)] airfoil sections."""
    wires = []
    for up, lo, plane, origin, dx in sections:
        shift = onp.array([dx, 0.0])
        wires.append(_outline_wp(cq, up + shift, lo + shift, plane, origin).val())
    return cq.Workplane(obj=cq.Solid.makeLoft(wires, True))


def build_tail_plate(spec: SurfaceSpec, te_mm: float):
    """Airfoil-section horizontal tail across both halves, lofted root -> tips.

    Built in the plane frame (x aft from the root LE, y right, z up); the airfoil is
    `spec.airfoil`. See `build_htail` for the stabilizer/elevator split.
    """
    cq = _cq()
    half = spec.span / 2 * 1000
    rc, tc = spec.root_chord * 1000, spec.tip_chord * 1000
    sweep = half * math.tan(math.radians(spec.quarter_chord_sweep_deg))
    x_le_tip = sweep + 0.25 * (rc - tc)
    up_r, lo_r = _airfoil_curves(spec.airfoil, rc, te_mm)
    up_t, lo_t = _airfoil_curves(spec.airfoil, tc, te_mm)
    # XZ plane normal is -y, so sections are placed with an explicit origin on y
    return _loft_sections(
        cq,
        [
            (up_t, lo_t, "XZ", (0, -half, 0), x_le_tip),
            (up_r, lo_r, "XZ", (0, 0, 0), 0.0),
            (up_t, lo_t, "XZ", (0, half, 0), x_le_tip),
        ],
    )


def build_htail(spec: SurfaceSpec, te_mm: float):
    """(stabilizer, elevator) airfoil solids, split along the hinge line with the hinge gap.

    Without a control surface this is `(build_tail_plate(...), None)`. The elevator is one
    piece across both halves (a single servo and horn drive it). Both pieces are cut from
    the full airfoil loft by a prism of the plan-view outline, so the hinge line is straight
    root to tip. The hinge faces are flat (the film/tape hinge crosses the gap).
    """
    full = build_tail_plate(spec, te_mm)
    cs = spec.control_surface
    if cs is None:
        return full, None
    cq = _cq()
    half = spec.span / 2 * 1000
    rc, tc = spec.root_chord * 1000, spec.tip_chord * 1000
    sweep = half * math.tan(math.radians(spec.quarter_chord_sweep_deg))
    x_le_tip = sweep + 0.25 * (rc - tc)
    g = cs.hinge_gap_mm / 2
    h0 = (1 - cs.chord_fraction) * rc  # hinge x at the root
    h1 = x_le_tip + (1 - cs.chord_fraction) * tc  # hinge x at the tips
    pad = 10.0  # cutter margin beyond the planform, mm
    tall = 50.0

    def prism(pts):
        return (
            cq.Workplane("XY")
            .workplane(offset=-tall / 2)
            .polyline(pts)
            .close()
            .extrude(tall)
        )

    stab = full.intersect(
        prism(
            [
                (-pad, -half - pad),
                (h1 - g, -half - pad),
                (h1 - g, -half),
                (h0 - g, 0),
                (h1 - g, half),
                (h1 - g, half + pad),
                (-pad, half + pad),
            ]
        )
    )
    elevator = full.intersect(
        prism(
            [
                (h1 + g, -half - pad),
                (rc + tc + pad, -half - pad),
                (rc + tc + pad, half + pad),
                (h1 + g, half + pad),
                (h1 + g, half),
                (h0 + g, 0),
                (h1 + g, -half),
            ]
        )
    )
    return stab, elevator


def build_fin(spec: FinSpec, te_mm: float):
    """Airfoil-section vertical fin, centred on y=0, root on z=0, extending +z."""
    cq = _cq()
    h = spec.height * 1000
    rc, tc = spec.root_chord * 1000, spec.tip_chord * 1000
    x_le_tip = h * math.tan(math.radians(spec.le_sweep_deg))
    up_r, lo_r = _airfoil_curves(spec.airfoil, rc, te_mm)
    up_t, lo_t = _airfoil_curves(spec.airfoil, tc, te_mm)
    return _loft_sections(
        cq,
        [
            (up_r, lo_r, "XY", (0, 0, 0), 0.0),
            (up_t, lo_t, "XY", (0, 0, h), x_le_tip),
        ],
    )


def export_all(d: Design, out_dir: Path, parts=None) -> dict[str, Path]:
    """Write per-rib STEP/STL, print plates (split to fit the printer bed), and the assembly."""
    cq = _cq()
    out_dir.mkdir(parents=True, exist_ok=True)
    parts = parts or build_ribs(d)
    paths: dict[str, Path] = {}
    for r, w in parts:
        for ext in ("step", "stl"):
            p = out_dir / f"rib_{r.index:02d}.{ext}"
            cq.exporters.export(w, str(p))
            paths[p.name] = p
    control_parts = build_control_ribs(d)
    for r, w in control_parts:
        for ext in ("step", "stl"):
            p = out_dir / f"aileron_rib_{r.index:02d}.{ext}"
            cq.exporters.export(w, str(p))
            paths[p.name] = p
    named = [(f"rib_{r.index:02d}", w) for r, w in parts]
    named += [(f"aileron_rib_{r.index:02d}", w) for r, w in control_parts]
    plates = pack_plates(named, d.printer)
    for i, plate in enumerate(plates, start=1):
        for ext in ("step", "stl"):
            p = out_dir / f"print_plate_{i:02d}.{ext}"
            cq.exporters.export(plate, str(p))
            paths[p.name] = p
    asm = wing_assembly(d, parts, control_parts)
    p = out_dir / "wing_half_assembly.step"
    asm.export(str(p), exportType="STEP")
    paths[p.name] = p

    # Nose pod, htail, vtail: printable/reference parts. The boom itself is stock
    # carbon tube (see boom_length_mm), not modeled or printed here.
    stab, elevator = build_htail(d.htail, d.printer.tail_te_mm)
    tail_parts = [
        ("fuselage_pod", build_pod(d)),
        ("htail", stab),
        ("vtail", build_fin(d.vtail, d.printer.tail_te_mm)),
    ]
    if elevator is not None:
        tail_parts.insert(2, ("elevator", elevator))
    for name, solid in tail_parts:
        for ext in ("step", "stl"):
            p = out_dir / f"{name}.{ext}"
            cq.exporters.export(solid, str(p))
            paths[p.name] = p

    full = full_aircraft_assembly(d, parts, control_parts)
    p = out_dir / "full_aircraft_assembly.step"
    full.export(str(p), exportType="STEP")
    paths[p.name] = p
    return paths
