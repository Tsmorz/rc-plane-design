"""Stability, trim, and performance with AeroSandbox's AeroBuildup (NeuralFoil inside)."""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import aerosandbox as asb
import aerosandbox.numpy as np

from .config import Design
from .geometry import make_airplane, make_wing
from .mass import cg, total_mass_g

warnings.filterwarnings("ignore", category=FutureWarning)

G = 9.81


def _op(d: Design, velocity, alpha) -> asb.OperatingPoint:
    return asb.OperatingPoint(
        atmosphere=asb.Atmosphere(altitude=d.field_altitude_m),
        velocity=velocity,
        alpha=alpha,
    )


def mac(d: Design) -> float:
    return float(make_wing(d.wing).mean_aerodynamic_chord())


def wing_area(d: Design) -> float:
    return float(make_wing(d.wing).area())


def neutral_point(d: Design, velocity: float = 10.0, alpha: float = 2.0) -> float:
    """Longitudinal neutral point x [m]. Depends on geometry only, not on CG."""
    res = asb.AeroBuildup(
        make_airplane(d),
        _op(d, velocity, alpha),
    ).run_with_stability_derivatives(alpha=True, beta=False, p=False, q=False, r=False)
    return float(np.atleast_1d(res["x_np"])[0])


def static_margin(d: Design) -> float:
    return (neutral_point(d) - cg(d)[0]) / mac(d)


@dataclass
class TrimPoint:
    """Trimmed flight condition at one airspeed."""

    velocity: float
    alpha_deg: float
    tail_incidence_deg: float
    CL: float
    CD: float
    L_over_D: float
    drag_N: float
    electrical_power_W: float  # ideal level flight
    flight_time_min: float  # ideal level flight
    realistic_flight_time_min: float


def trim(d: Design, velocity: float, usable_battery_fraction: float = 0.8) -> TrimPoint:
    """Level flight: solve alpha and tail incidence so that L = W and Cm = 0 about the CG."""
    weight = total_mass_g(d) / 1000 * G
    x_cg, z_cg = cg(d)

    opti = asb.Opti()
    alpha = opti.variable(init_guess=2.0, lower_bound=-6, upper_bound=14)
    tail_inc = opti.variable(init_guess=-1.0, lower_bound=-12, upper_bound=12)
    res = asb.AeroBuildup(
        make_airplane(d, xyz_ref=(x_cg, 0, z_cg), tail_incidence_deg=tail_inc),
        _op(d, velocity, alpha),
    ).run()
    opti.subject_to([res["L"] == weight, res["Cm"] == 0])
    sol = opti.solve(verbose=False)

    drag = float(sol(res["D"]))
    p_elec = drag * velocity / d.propulsive_efficiency
    return TrimPoint(
        velocity=velocity,
        alpha_deg=float(sol(alpha)),
        tail_incidence_deg=float(sol(tail_inc)),
        CL=float(sol(res["CL"])),
        CD=float(sol(res["CD"])),
        L_over_D=float(sol(res["L"] / res["D"])),
        drag_N=drag,
        electrical_power_W=p_elec,
        flight_time_min=d.battery_wh * usable_battery_fraction / p_elec * 60,
        realistic_flight_time_min=d.battery_wh
        * usable_battery_fraction
        / (p_elec * d.real_world_power_factor)
        * 60,
    )


def cl_max(d: Design, velocity: float = 7.0) -> float:
    """Whole-airplane CLmax from an alpha sweep (NeuralFoil captures post-stall trends)."""
    x_cg, z_cg = cg(d)
    airplane = make_airplane(d, xyz_ref=(x_cg, 0, z_cg))
    alphas = np.linspace(0, 18, 37)
    cls = [
        float(
            np.atleast_1d(asb.AeroBuildup(airplane, _op(d, velocity, a)).run()["CL"])[0]
        )
        for a in alphas
    ]
    return max(cls)


def stall_speed(d: Design) -> float:
    rho = asb.Atmosphere(altitude=d.field_altitude_m).density()
    w = total_mass_g(d) / 1000 * G
    return float(np.sqrt(2 * w / (rho * wing_area(d) * cl_max(d))))


@dataclass
class ElevatorTrim:
    """Elevator needed to hold level flight at one airspeed, stabilizer fixed."""

    velocity: float
    alpha_deg: float
    elevator_deg: float  # positive = trailing edge down
    authority_used: float  # |deflection| / mechanical travel


def trim_elevator(d: Design, velocity: float) -> ElevatorTrim:
    """Level flight with the stab fixed at `htail.root_incidence_deg`; solve alpha and elevator.

    This is what the pilot actually flies: the stabilizer is built at one incidence and
    the elevator supplies the rest of the trim across airspeed.
    """
    cs = d.htail.control_surface
    if cs is None:
        raise ValueError("htail has no control surface")
    weight = total_mass_g(d) / 1000 * G
    x_cg, z_cg = cg(d)

    opti = asb.Opti()
    alpha = opti.variable(init_guess=2.0, lower_bound=-6, upper_bound=14)
    elev = opti.variable(init_guess=0.0, lower_bound=-30, upper_bound=30)
    airplane = make_airplane(d, xyz_ref=(x_cg, 0, z_cg)).with_control_deflections(
        {cs.name: elev}
    )
    res = asb.AeroBuildup(airplane, _op(d, velocity, alpha)).run()
    opti.subject_to([res["L"] == weight, res["Cm"] == 0])
    sol = opti.solve(verbose=False)
    e = float(sol(elev))
    return ElevatorTrim(velocity, float(sol(alpha)), e, abs(e) / cs.max_deflection_deg)


def roll_rate_dps(
    d: Design, velocity: float, deflection_deg: float | None = None
) -> float:
    """Steady-state roll rate [deg/s] for an aileron deflection (default: full travel).

    Evaluated at the level-flight alpha for that speed. Balances the aileron rolling moment against roll damping: p = -Cl_da * da / Clp
    scaled by 2V/b. Ignores adverse yaw and roll-up time, so it is an upper bound.
    """
    cs = d.wing.control_surface
    if cs is None:
        raise ValueError("wing has no control surface")
    da = cs.max_deflection_deg if deflection_deg is None else deflection_deg
    x_cg, z_cg = cg(d)
    airplane = make_airplane(d, xyz_ref=(x_cg, 0, z_cg))
    op = _op(d, velocity, trim(d, velocity).alpha_deg)  # level-flight alpha
    cl_ail = float(
        np.atleast_1d(
            asb.AeroBuildup(airplane.with_control_deflections({cs.name: da}), op).run()[
                "Cl"
            ]
        )[0]
    )
    clp = float(
        np.atleast_1d(
            asb.AeroBuildup(airplane, op).run_with_stability_derivatives(
                alpha=False, beta=False, p=True, q=False, r=False
            )["Clp"]
        )[0]
    )
    p_hat = -cl_ail / clp  # p * b / (2V), radians
    return abs(float(np.degrees(p_hat * 2 * velocity / d.wing.span)))
