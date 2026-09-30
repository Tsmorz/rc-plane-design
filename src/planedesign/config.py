"""Design parameters: the single source of truth for the airplane.

Coordinate system (AeroSandbox geometry axes):
    x: aft, measured from the wing root leading edge [m]
    y: right wing [m]
    z: up [m]

Edit this file, then rerun the scripts. Everything downstream (mass budget,
stability, trim, rib export) reads from here.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# --- Legal / budget limits -------------------------------------------------
MAX_TAKEOFF_MASS_G = 250.0  # hard legal ceiling (EU/Germany sub-250 g)
TARGET_TAKEOFF_MASS_G = 225.0  # design target, leaves margin for build overshoot


@dataclass
class SurfaceSpec:
    """A symmetric, tapered lifting surface (wing or horizontal tail)."""

    name: str
    airfoil: str  # name in the UIUC database, e.g. "sd7062"
    span: float  # tip-to-tip [m]
    root_chord: float  # [m]
    tip_chord: float  # [m]
    x_le_root: float = 0.0  # root leading-edge x position [m]
    z_root: float = 0.0  # root z position [m]
    dihedral_deg: float = 0.0
    root_incidence_deg: float = 0.0
    washout_deg: float = 0.0  # tip twist relative to root, positive = nose-down
    quarter_chord_sweep_deg: float = 0.0


@dataclass
class FinSpec:
    """A single vertical fin on the centerline."""

    airfoil: str
    height: float
    root_chord: float
    tip_chord: float
    x_le_root: float
    z_root: float = 0.0
    le_sweep_deg: float = 20.0


@dataclass
class Component:
    """A point mass in the mass budget."""

    name: str
    mass_g: float
    x: float  # CG location [m]
    z: float = 0.0
    note: str = ""


@dataclass
class Design:
    """Complete airplane definition: surfaces, airfoils, and mass items."""

    wing: SurfaceSpec
    htail: SurfaceSpec
    vtail: FinSpec
    components: list[Component]
    # Fuselage as (x, radius) stations, approximates pod + boom for drag
    fuselage_stations: list[tuple[float, float]] = field(default_factory=list)
    battery_name: str = "battery"  # component moved by the CG solver
    target_static_margin: float = 0.15  # fraction of MAC; trainer: 0.12-0.20
    battery_wh: float = 2 * 3.7 * 0.450  # 2S 450 mAh
    propulsive_efficiency: float = 0.45  # motor x ESC x prop, small-plane estimate
    rib_spacing: float = 0.050  # wing rib pitch [m]
    field_altitude_m: float = 500.0  # flying-site elevation, sets air density
    # AeroBuildup predicts ideal steady level flight on a perfect surface. Climbs, turns,
    # gusts, film scalloping between ribs and interference drag typically cost 2-3x more.
    real_world_power_factor: float = 2.5


DESIGN = Design(
    wing=SurfaceSpec(
        name="Wing",
        airfoil="sd7062",  # decided: final wing airfoil
        span=0.80,
        z_root=0.022,  # high wing, sits on top of the pod
        root_chord=0.180,
        tip_chord=0.130,
        dihedral_deg=3.0,
        root_incidence_deg=2.0,
        washout_deg=1.0,
    ),
    htail=SurfaceSpec(
        name="Horizontal Stabilizer",
        airfoil="naca0006",  # stands in for a flat Depron plate
        span=0.220,
        root_chord=0.085,
        tip_chord=0.065,
        x_le_root=0.450,
        z_root=0.0,
    ),
    vtail=FinSpec(
        airfoil="naca0006",
        height=0.110,
        root_chord=0.100,
        tip_chord=0.070,
        x_le_root=0.460,
    ),
    fuselage_stations=[
        (-0.125, 0.012),  # spinner / motor
        (-0.090, 0.022),
        (0.120, 0.020),
        (0.200, 0.008),
        (0.550, 0.004),  # end of tail boom
    ],
    components=[
        Component("motor_prop", 33.0, -0.125, note="DX2205 2300KV + 5in prop"),
        Component("esc", 8.0, -0.090),
        Component("battery", 32.0, 0.085, note="2S 450 mAh; x solved for CG"),
        Component("wiring", 10.0, -0.030, note="leads, XT30, servo extensions"),
        Component("fuselage_spine", 16.0, 0.000, note="Onyx + fiberglass spine plate"),
        Component("flight_controller", 10.0, 0.030, note="ESP32-S3 + IMU/mag/baro"),
        Component("receiver_bec", 4.0, 0.060),
        Component("gps", 6.0, 0.080, z=0.03),
        Component("hardware", 6.0, 0.050, note="glue, screws, hinge tape"),
        Component("wing_structure", 58.0, 0.070, note="ribs, spars, film (~40% chord)"),
        Component("servos_aileron", 9.0, 0.070, note="2x 4.5 g, in wing"),
        Component("servo_elevator", 4.5, 0.100),
        Component("boom", 5.0, 0.300, note="carbon tube"),
        Component("htail", 7.0, 0.490, note="Depron + film"),
        Component("vtail", 4.0, 0.500),
    ],
)
