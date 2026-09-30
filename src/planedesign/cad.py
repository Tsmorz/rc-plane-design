"""CadQuery rib generator: printable ribs, a print plate, and a wing-half assembly.

Each rib is built flat in the XY plane (chord along +x, leading edge at the origin,
extruded +z) so it prints lying on the bed. Features added on top of the bare airfoil:

- trailing edge cut back to `te_min_mm` so it is printable
- square spar hole on the camber line (collinear across ribs, so a straight tube fits)
- open slot through the nose for the LE carbon rod (snaps in from the front)
- Warren-truss lightening (diagonal members inside a solid rim) fore and aft of the spar

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

from .config import Design, RibSpec
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


def _outline_wp(cq, up: onp.ndarray, lo: onp.ndarray):
    """Build the closed rib outline: upper spline, TE cut, lower spline back to the LE."""
    w = cq.Workplane("XY").moveTo(*up[0])
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


def build_rib(r: Rib, spec: RibSpec):
    """One rib as a CadQuery Workplane (flat, extruded +z)."""
    cq = _cq()
    c_mm = r.chord * 1000
    upper, lower = _surfaces(r.outline * 1000)
    t_rib = spec.thickness_mm

    # --- trim the trailing edge back to te_min ---------------------------
    xs = onp.linspace(0.5 * c_mm, c_mm, 400)
    thick = _z(upper, xs) - _z(lower, xs)
    ok = onp.where(thick >= spec.te_min_mm)[0]
    x_cut = float(xs[ok[-1]]) if len(ok) else c_mm
    up = upper[upper[:, 0] < x_cut]
    lo = lower[lower[:, 0] < x_cut]
    up = onp.vstack([up, [x_cut, _z(upper, x_cut)]])
    lo = onp.vstack([lo, [x_cut, _z(lower, x_cut)]])

    rib = _outline_wp(cq, up, lo).extrude(t_rib)
    inset = _to_nurbs(_outline_wp(cq, up, lo).offset2D(-spec.min_web_mm).extrude(t_rib))

    sx, sz = r.spar_center[0] * 1000, r.spar_center[1] * 1000
    hole_w = spec.spar_width_mm + spec.spar_clearance_mm
    th = math.radians(r.twist_deg)
    # Square sides are level in the world, which is the rib frame rotated by +twist.
    corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    hole_pts = [
        (
            sx + (cx * math.cos(th) - cz * math.sin(th)) * hole_w / 2,
            sz + (cx * math.sin(th) + cz * math.cos(th)) * hole_w / 2,
        )
        for cx, cz in corners
    ]
    rod_r = spec.le_rod_mm / 2 + 0.1
    rod_x = spec.le_rod_mm / 2 + 0.2

    # --- truss lightening: nose bay ahead of the spar, then aft to the TE ---
    boss = (
        cq.Workplane("XY")
        .center(sx, sz)
        .circle(hole_w * math.sqrt(2) / 2 + spec.min_web_mm)
        .extrude(t_rib)
    )
    x_front = rod_x + rod_r + spec.min_web_mm
    x_back = x_cut - spec.min_web_mm
    for x0, x1 in ((x_front, sx), (sx, x_back)):
        for p in _truss_pockets(cq, upper, lower, inset, x0, x1, spec):
            rib = rib.cut(p.cut(boss))

    # --- square spar hole, rotated to the rib's twist -----------------------
    rib = rib.cut(cq.Workplane("XY").polyline(hole_pts).close().extrude(t_rib))

    # --- LE rod slot: rod-diameter channel from in front of the nose inward --
    rod_z = 0.5 * (_z(upper, rod_x) + _z(lower, rod_x))
    rib = rib.cut(
        cq.Workplane("XY")
        .center(rod_x, rod_z)
        .circle(rod_r)
        .extrude(t_rib)
        .union(
            cq.Workplane("XY")
            .center(-1.0, rod_z)
            .rect(2 * (rod_x + 1.0), 2 * rod_r)
            .extrude(t_rib)
        )
    )
    return rib


def build_ribs(d: Design):
    """[(Rib, Workplane)] for one wing half."""
    return [(r, build_rib(r, d.rib)) for r in rib_stations(d)]


def print_plate(parts, gap_mm: float = 6.0):
    """Lay all ribs flat on one plate in rows for a single Eiger job."""
    cq = _cq()
    plate = cq.Workplane("XY")
    y_cursor = 0.0
    solids = []
    for _, w in parts:
        bb = w.val().BoundingBox()
        moved = w.translate((-bb.xmin, y_cursor - bb.ymin, 0))
        solids.append(moved.val())
        y_cursor += (bb.ymax - bb.ymin) + gap_mm
    return plate.newObject(solids)


def wing_assembly(d: Design, parts):
    """Ribs stood up at their span stations, twisted about the spar, plus the spar."""
    cq = _cq()
    t_mm = d.rib.thickness_mm
    asm = cq.Assembly(name="wing_half")
    centers = []
    for r, w in parts:
        sx, sz = r.spar_center[0] * 1000, r.spar_center[1] * 1000
        # flat (x, z_airfoil, thickness) -> world (x, thickness, z_airfoil)
        s = w.rotate((0, 0, 0), (1, 0, 0), 90).translate((0, r.y * 1000 + t_mm, 0))
        s = s.rotate((sx, 0, sz), (sx, 1, sz), r.twist_deg)
        asm.add(s, name=f"rib_{r.index:02d}")
        centers.append((sx, r.y * 1000 + t_mm / 2, sz))
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


def export_all(d: Design, out_dir: Path, parts=None) -> dict[str, Path]:
    """Write per-rib STEP/STL, the print plate, and the assembly."""
    cq = _cq()
    out_dir.mkdir(parents=True, exist_ok=True)
    parts = parts or build_ribs(d)
    paths: dict[str, Path] = {}
    for r, w in parts:
        for ext in ("step", "stl"):
            p = out_dir / f"rib_{r.index:02d}.{ext}"
            cq.exporters.export(w, str(p))
            paths[p.name] = p
    plate = print_plate(parts)
    for ext in ("step", "stl"):
        p = out_dir / f"print_plate.{ext}"
        cq.exporters.export(plate, str(p))
        paths[p.name] = p
    asm = wing_assembly(d, parts)
    p = out_dir / "wing_half_assembly.step"
    asm.export(str(p), exportType="STEP")
    paths[p.name] = p
    return paths
