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
class ControlSurfaceSpec:
    """A trailing-edge control surface (aileron or elevator) on a `SurfaceSpec`.

    The hinge sits at `1 - chord_fraction` of the local chord and runs straight between
    `span_start` and `span_end`. Ailerons (`symmetric=False`) deflect differentially, so
    left and right move opposite ways; the elevator (`symmetric=True`) moves as one.
    """

    name: str
    chord_fraction: float  # control surface chord / local chord
    span_start: float = 0.0  # inboard end, fraction of semi-span
    span_end: float = 1.0  # outboard end, fraction of semi-span
    symmetric: bool = True
    max_deflection_deg: float = 25.0  # mechanical travel each way
    hinge_gap_mm: float = 1.0  # gap cut across the hinge line for tape/film hinge


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
    control_surface: ControlSurfaceSpec | None = None


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
    # Set this once the part is on the scale (0.1 g resolution); the budget, CG and
    # tests then use it in place of the estimate in `mass_g`.
    measured_g: float | None = None

    @property
    def actual_g(self) -> float:
        """Measured mass if weighed, otherwise the estimate."""
        return self.mass_g if self.measured_g is None else self.measured_g


@dataclass
class PowerItem:
    """An electrical load in the power budget (all values at the battery, in watts)."""

    name: str
    typical_w: float  # average draw in steady cruise
    peak_w: float  # worst-case simultaneous draw (full throttle, servo stall, radio TX)
    note: str = ""


@dataclass
class PrinterSpec:
    """Build volume of the printer, for laying out and splitting print plates."""

    bed_width_mm: float = 320.0
    bed_depth_mm: float = 132.0
    bed_height_mm: float = 154.0
    plate_gap_mm: float = 6.0  # clearance between parts on a plate
    # Tail surfaces are printed airfoils (Design.htail / vtail airfoil). The trailing edge is
    # thickened to this, blended in linearly from the nose, so it survives printing.
    tail_te_mm: float = 0.8


@dataclass
class RibSpec:
    """Printed-rib features for the CadQuery generator (all in mm).

    Defaults are conservative Onyx-on-Markforged values; tune them to your printer.
    """

    thickness_mm: float = 1.2  # extrusion thickness of each flat rib
    te_min_mm: float = 1.0  # trailing edge is cut back to at least this thickness
    spar_width_mm: float = 3.0  # square carbon tube outer width
    spar_clearance_mm: float = 0.2  # added to the spar hole diameter for glue
    spar_web_mm: float = 2.0  # solid full-depth web each side of the spar hole
    spar_collar_mm: float = 3.0  # sleeve standing off the rib face (spar engagement)
    spar_collar_wall_mm: float = 1.2  # sleeve wall around the spar hole
    le_rod_mm: float = 1.0  # leading-edge carbon rod diameter (snap-in notch)
    min_web_mm: float = 1.5  # minimum material between holes and the skin
    min_hole_mm: float = 3.0  # pockets smaller than about this are left solid
    truss_member_mm: float = 1.5  # width of truss diagonals and posts
    truss_bay_ratio: float = 1.2  # bay length / local thickness; sets diagonal angle
    density_g_cc: float = 1.2  # Onyx, approximate; check the datasheet


@dataclass
class Design:
    """Complete airplane definition: surfaces, airfoils, and mass items."""

    wing: SurfaceSpec
    htail: SurfaceSpec
    vtail: FinSpec
    components: list[Component]
    # Fuselage as (x, radius) stations, approximates pod + boom for drag
    fuselage_stations: list[tuple[float, float]] = field(default_factory=list)
    # Only stations up to here are printed as the nose pod; aft of it the fuselage is a
    # constant-diameter carbon tube boom (stock, not printed) running to the tail mount,
    # matching the "boom" mass component below.
    pod_end_x: float = 0.200
    boom_od_mm: float = 6.0  # off-the-shelf carbon tube OD
    battery_name: str = "battery"  # component moved by the CG solver
    target_static_margin: float = 0.15  # fraction of MAC; trainer: 0.12-0.20
    battery_wh: float = 2 * 3.7 * 0.450  # 2S 450 mAh
    battery_nominal_v: float = 7.4  # 2S
    battery_c_rating: float = 45.0  # PLACEHOLDER: use your pack's continuous C rating
    battery_usable_fraction: float = 0.8  # land with 20 % left, protects the pack
    # Electrical loads. Values are estimates: replace with bench measurements (a USB
    # power meter on the bench supply, or logged flight current) as you get them.
    power_items: list[PowerItem] = field(default_factory=list)
    propulsive_efficiency: float = 0.45  # motor x ESC x prop, small-plane estimate
    rib_spacing: float = 0.050  # wing rib pitch [m]
    rib: RibSpec = field(default_factory=RibSpec)
    printer: PrinterSpec = field(default_factory=PrinterSpec)
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
        # Aileron ends sit on rib stations (0.5 and 1.0 of the semi-span at the default
        # 50 mm pitch): those two ribs stay whole as closeouts, the ribs between them
        # are split at the hinge (see cad.build_control_ribs).
        control_surface=ControlSurfaceSpec(
            "aileron",
            chord_fraction=0.28,
            span_start=0.5,
            span_end=1.0,
            symmetric=False,
            max_deflection_deg=20.0,
        ),
    ),
    htail=SurfaceSpec(
        name="Horizontal Stabilizer",
        airfoil="naca0006",  # stands in for a flat Depron plate
        span=0.220,
        root_chord=0.085,
        tip_chord=0.065,
        x_le_root=0.450,
        z_root=0.0,
        control_surface=ControlSurfaceSpec(
            "elevator", chord_fraction=0.35, symmetric=True, max_deflection_deg=25.0
        ),
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
    power_items=[
        PowerItem(
            "motor", 20.0, 90.0, "cruise from 02_analyze_design; peak = WOT static"
        ),
        PowerItem(
            "flight_controller", 0.6, 1.2, "ESP32-S3 + IMU/mag/baro, WiFi TX spikes"
        ),
        PowerItem("gps", 0.15, 0.25),
        PowerItem("receiver_bec", 0.3, 0.5, "BEC + receiver losses"),
        PowerItem(
            "servos_aileron", 0.4, 5.0, "2x 4.5 g servos, peak = both moving hard"
        ),
        PowerItem("servo_elevator", 0.2, 2.5),
        PowerItem("esc", 0.1, 0.5, "quiescent + switching losses"),
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
