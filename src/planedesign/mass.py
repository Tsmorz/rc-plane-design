"""Mass budget and CG."""
from __future__ import annotations

from dataclasses import replace

from .config import MAX_TAKEOFF_MASS_G, TARGET_TAKEOFF_MASS_G, Component, Design


def total_mass_g(d: Design) -> float:
    return sum(c.mass_g for c in d.components)


def cg(d: Design) -> tuple[float, float]:
    """(x_cg, z_cg) in meters."""
    m = total_mass_g(d)
    return (
        sum(c.mass_g * c.x for c in d.components) / m,
        sum(c.mass_g * c.z for c in d.components) / m,
    )


def battery_x_for_cg(d: Design, x_cg_target: float) -> float:
    """Battery x position that puts the CG at x_cg_target."""
    bat = next(c for c in d.components if c.name == d.battery_name)
    m = total_mass_g(d)
    others = sum(c.mass_g * c.x for c in d.components if c is not bat)
    return (x_cg_target * m - others) / bat.mass_g


def with_battery_at(d: Design, x: float) -> Design:
    comps = [replace(c, x=x) if c.name == d.battery_name else c for c in d.components]
    return replace(d, components=comps)


def budget_table(d: Design) -> str:
    rows = sorted(d.components, key=lambda c: -c.mass_g)
    m = total_mass_g(d)
    lines = [f"{'component':<20}{'g':>7}{'x [mm]':>9}  note", "-" * 60]
    for c in rows:
        lines.append(f"{c.name:<20}{c.mass_g:7.1f}{c.x * 1000:9.0f}  {c.note}")
    lines.append("-" * 60)
    lines.append(f"{'TOTAL':<20}{m:7.1f}")
    lines.append(f"margin to target ({TARGET_TAKEOFF_MASS_G:.0f} g): {TARGET_TAKEOFF_MASS_G - m:+.1f} g")
    lines.append(f"margin to legal  ({MAX_TAKEOFF_MASS_G:.0f} g): {MAX_TAKEOFF_MASS_G - m:+.1f} g")
    return "\n".join(lines)


def add_component(d: Design, comp: Component) -> Design:
    """Handy for what-ifs, e.g. adding a camera."""
    return replace(d, components=[*d.components, comp])
