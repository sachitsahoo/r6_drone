"""Parametric solids for the Recon UGV mechanical parts.

Every dimension comes from parameters.py; there are no literals here beyond counts and
trivial fractions. Seven parts, built from four shape types -- tube, disc, flat plate,
solid of revolution.

Direct drive (ADR 0008) deleted four parts from the first design: the drive band, the
pulley, the belt tensioner slots and the motor bracket. The chassis discs and standoffs went
too, replaced by a single spine on the axis (ADR 0010).

This is a STARTING POINT, not a finished design. Several dimensions in parameters.py are
tagged ASSUMPTION because the design needed a number and no analysis supplies one. Read that
file before printing anything.

Run `python3 cad/build.py` to export STEP and STL and print mass properties.
"""

from __future__ import annotations

import math

import cadquery as cq

import parameters as P


def _ring_points(radius: float, count: int, phase_deg: float = 0.0) -> list[tuple[float, float]]:
    """Evenly spaced points on a circle, for bolt patterns."""
    return [
        (radius * math.cos(math.radians(phase_deg + i * 360.0 / count)),
         radius * math.sin(math.radians(phase_deg + i * 360.0 / count)))
        for i in range(count)
    ]


def _revolve_profile(points_rz: list[tuple[float, float]]) -> cq.Workplane:
    """Solid of revolution about Z from a closed (radius, z) outline.

    On the XZ workplane local x is global X and local y is global Z, so the outline is drawn
    in (r, z) directly and revolved about the local y axis, which is the global Z axis.
    """
    return (cq.Workplane("XZ").polyline(points_rz).close()
            .revolve(360.0, (0, 0, 0), (0, 1, 0)))


# --------------------------------------------------------------------------- part 1

def casing_shell() -> cq.Workplane:
    """The rotating outer casing: a tube, plus an aperture and mounting features.

    Shape: cylinder minus cylinder. Features: camera aperture, internal end flanges for
    the end cap screws, and trim mass bosses (R4).
    """
    r_out = P.CASING_OD / 2
    r_in = P.CASING_ID / 2
    length = P.CASING_LENGTH
    flange_depth = P.SHELL_FLANGE_DEPTH

    shell = cq.Workplane("XY").circle(r_out).circle(r_in).extrude(length)

    # Internal flanges at both ends: the wall is too thin for axial tapped holes, so
    # thicken it locally rather than thickening the whole shell (which would cost inertia
    # at the largest radius -- see R6).
    flange_r_in = r_in - P.SHELL_FLANGE_RADIAL
    for z in (0.0, length - flange_depth):
        flange = (cq.Workplane("XY", origin=(0, 0, z))
                  .circle(r_in).circle(flange_r_in).extrude(flange_depth))
        shell = shell.union(flange)

    # End cap screws: tapped axially into the flanges.
    for z, direction in ((0.0, 1.0), (length, -1.0)):
        plane = cq.Workplane("XY", origin=(0, 0, z))
        holes = (plane.pushPoints(_ring_points(P.END_CAP_SCREW_RADIUS, P.END_CAP_SCREW_COUNT))
                 .circle(P.M3_TAP / 2)
                 .extrude(direction * flange_depth))
        shell = shell.cut(holes)

    # Camera aperture, cut radially through the wall at the camera station.
    aperture = (cq.Workplane("YZ", origin=(0, 0, P.Z_CAMERA))
                .rect(P.CAMERA_APERTURE_WIDTH, P.CAMERA_APERTURE_HEIGHT)
                .extrude(r_out + 1.0))
    shell = shell.cut(aperture)

    # Trim mass bosses (R4): radial pads on the inner wall with tapped holes, so the
    # balanced-vs-bottom-heavy decision stays open after assembly.
    # A YZ workplane extrudes along +X from wherever its origin sits, so the plane starts
    # just inside the wall -- starting at x=0 once produced six solid rods across the bore.
    # The tap stays blind so nothing pierces the impact surface.
    boss_x0 = r_in - P.TRIM_BOSS_HEIGHT
    for x, y in _ring_points(1.0, P.TRIM_BOSS_COUNT):          # unit vector per position
        angle = math.degrees(math.atan2(y, x))
        pad = (cq.Workplane("YZ", origin=(boss_x0, 0, P.Z_TRIM_BOSSES))
               .circle(P.TRIM_BOSS_OD / 2)
               .extrude(P.TRIM_BOSS_HEIGHT)
               .rotate((0, 0, 0), (0, 0, 1), angle))
        tap = (cq.Workplane("YZ", origin=(boss_x0, 0, P.Z_TRIM_BOSSES))
               .circle(P.M3_TAP / 2)
               .extrude(P.TRIM_BOSS_HEIGHT - 1.0)      # blind: does not reach the wall
               .rotate((0, 0, 0), (0, 0, 1), angle))
        shell = shell.union(pad).cut(tap)

    return shell


# --------------------------------------------------------------------------- part 2

def casing_end_cap_a() -> cq.Workplane:
    """The motor-end cap: the pitch motor's stator bolts to it, and it rides on the cup.

    Shape: disc, stepped. z = 0 is the outboard face, which seats against the shell flange.
    Features: an annular groove in the inboard face holding the 6708 bearing's outer race
    (the cup wall enters the groove and carries the inner race), the stator bolt circle, and
    a centre bore that wheel A's shaft passes through.

    This is where the motor's reaction torque enters the casing, so the hub inside the groove
    is the torque path: stator -> bolts -> hub -> shoulder under the groove -> rim -> shell.
    The stator is on this side, not the rotor, so the phase leads stay in the casing (R3).
    """
    r_out = P.CASING_ID / 2
    t = P.END_CAP_A_THICKNESS
    rim_t = P.END_CAP_RIM_THICKNESS
    housing_r = P.END_CAP_A_GROOVE_OUTER_R + P.END_CAP_A_HOUSING_WALL

    rim = cq.Workplane("XY").circle(r_out).circle(housing_r).extrude(rim_t)
    body = cq.Workplane("XY").circle(housing_r).extrude(t)
    cap = rim.union(body)

    cap = cap.cut(
        cq.Workplane("XY", origin=(0, 0, t - P.END_CAP_A_GROOVE_DEPTH))
        .circle(P.END_CAP_A_GROOVE_OUTER_R).circle(P.END_CAP_A_GROOVE_INNER_R)
        .extrude(P.END_CAP_A_GROOVE_DEPTH))

    cap = cap.cut(cq.Workplane("XY").circle(P.END_CAP_A_SHAFT_BORE / 2).extrude(t))

    cap = cap.cut(
        cq.Workplane("XY")
        .pushPoints(_ring_points(P.PITCH_MOTOR_STATOR_BOLT_RADIUS, P.PITCH_MOTOR_BOLT_COUNT,
                                 phase_deg=45.0))
        .circle(P.M2_CLEARANCE / 2).extrude(t))

    cap = cap.cut(
        cq.Workplane("XY")
        .pushPoints(_ring_points(P.END_CAP_SCREW_RADIUS, P.END_CAP_SCREW_COUNT))
        .circle(P.M3_CLEARANCE / 2).extrude(rim_t))
    return cap


# --------------------------------------------------------------------------- part 3

def casing_end_cap_b() -> cq.Workplane:
    """The plain end cap: rim, hub and spokes, stepped in thickness, with the pendulum datum.

    Shape: disc. Features: bearing seat in the hub for the 6704 riding on the spine's boss,
    screw holes matching the shell's flanges, and the pendulum datum hole (R8).

    Built as rim + hub + spokes because mass at the cap's outer radius is the most expensive
    mass in the rotating assembly. Stepped because the full thickness is needed only where
    the bearing seats; the rim and spokes carry screw loads and nothing else.
    """
    r_out = P.CASING_ID / 2
    hub_t = P.END_CAP_THICKNESS
    rim_t = P.END_CAP_RIM_THICKNESS
    rim_inner = r_out - 7.0
    hub_outer = P.BEARING_OD / 2 + 3.5
    spoke_count, spoke_width = 6, 9.0

    # z = 0 is the outboard face, which seats flat against the shell's end flange. The hub
    # is the only thing that protrudes inboard, so the seating face stays planar.
    rim = cq.Workplane("XY").circle(r_out).circle(rim_inner).extrude(rim_t)
    hub = cq.Workplane("XY").circle(hub_outer).extrude(hub_t)

    spokes = cq.Workplane("XY")
    for x, y in _ring_points(1.0, spoke_count):
        angle = math.degrees(math.atan2(y, x))
        spokes = spokes.union(
            cq.Workplane("XY").rect(r_out * 2, spoke_width).extrude(rim_t)
            .rotate((0, 0, 0), (0, 0, 1), angle))
    spokes = spokes.intersect(cq.Workplane("XY").circle(rim_inner).extrude(rim_t))

    cap = rim.union(hub).union(spokes)

    # Bearing seat, cut into the hub's inboard face.
    cap = cap.cut(
        cq.Workplane("XY", origin=(0, 0, hub_t - P.BEARING_WIDTH))
        .circle(P.BEARING_OD / 2).extrude(P.BEARING_WIDTH))
    # Clearance bore for the spine's boss, through the remaining shoulder.
    cap = cap.cut(
        cq.Workplane("XY").circle(P.END_CAP_BOSS_CLEARANCE_BORE / 2).extrude(hub_t))

    cap = cap.cut(
        cq.Workplane("XY")
        .pushPoints(_ring_points(P.END_CAP_SCREW_RADIUS, P.END_CAP_SCREW_COUNT))
        .circle(P.M3_CLEARANCE / 2).extrude(rim_t))

    cap = cap.cut(
        cq.Workplane("XY")
        .pushPoints(_ring_points(P.PENDULUM_DATUM_RADIUS, 1, P.PENDULUM_DATUM_ANGLE_DEG))
        .circle(P.PENDULUM_DATUM_DIA / 2).extrude(rim_t))
    return cap


# --------------------------------------------------------------------------- part 4

def chassis_spine() -> cq.Workplane:
    """The whole chassis: one solid of revolution on the axis, built in casing coordinates.

    Unlike the other parts this is built at its assembled z (casing face A = 0), because
    every one of its stations is defined in that frame in parameters.py. From end A:

      cup wall    carries the 6708's inner race, and surrounds the pitch motor
      cup floor   the motor's rotor bell bolts here; its bore lets wheel motor A in
      pocket A    wheel motor A, shaft outboard through the pitch motor's hollow bore
      waist       a 5 mm rod; the IMU ring sits around it (ADR 0010)
      pocket B    wheel motor B, shaft outboard through the boss
      boss B      carries cap B's 6704

    Why a spine at all (ADR 0010): the casing turns all the way round relative to the
    chassis, so every chassis feature sweeps a full ring. Anything the chassis has at
    radius r, at axial station z, forbids casing contents at that r and z. A spine keeps
    the chassis at the smallest radius possible and leaves the annulus to the casing.

    NOT a printable part as drawn. In reality it is two printed ends joined by a bought rod
    at the waist; it is one solid here because the interference sweep only cares about the
    space it occupies.
    """
    r_cup_o, r_cup_i = P.CUP_OD / 2, P.CUP_ID / 2
    r_tube, r_bore = P.SPINE_OD / 2, P.SPINE_BORE / 2
    r_waist = P.SPINE_WAIST_OD / 2
    r_boss = P.AXIS_BOSS_OD / 2

    outline = [
        (r_cup_i, P.Z_CUP_TIP),
        (r_cup_o, P.Z_CUP_TIP),
        (r_cup_o, P.Z_CUP_FLOOR_HI),
        (r_tube, P.Z_CUP_FLOOR_HI),
        (r_tube, P.Z_WAIST_LO),
        (r_waist, P.Z_WAIST_LO),
        (r_waist, P.Z_WAIST_HI),
        (r_tube, P.Z_WAIST_HI),
        (r_tube, P.Z_BOSS_B_LO),
        (r_boss, P.Z_BOSS_B_LO),
        (r_boss, P.Z_SPINE_END),
        (r_bore, P.Z_SPINE_END),
        (r_bore, P.Z_WHEEL_MOTOR_B_LO),
        (0.0, P.Z_WHEEL_MOTOR_B_LO),
        (0.0, P.Z_WHEEL_MOTOR_A_HI),
        (r_bore, P.Z_WHEEL_MOTOR_A_HI),
        (r_bore, P.Z_MOTOR_HI),
        (r_cup_i, P.Z_MOTOR_HI),
    ]
    spine = _revolve_profile(outline)

    rotor_bolts = (
        cq.Workplane("XY", origin=(0, 0, P.Z_MOTOR_HI))
        .pushPoints(_ring_points(P.PITCH_MOTOR_ROTOR_BOLT_RADIUS, P.PITCH_MOTOR_BOLT_COUNT,
                                 phase_deg=45.0))
        .circle(P.M2_CLEARANCE / 2).extrude(P.CUP_FLOOR_THICKNESS))
    return spine.cut(rotor_bolts)


# --------------------------------------------------------------------------- part 5

def imu_bridge() -> cq.Workplane:
    """Arm from the casing wall to a carrier plate that holds the IMU breakout beside the
    spine's waist (R2, ADR 0010).

    Shape: flat strip plus flat plate. Built with the rotation axis along Z through the
    origin and the board on +X; the assembly turns it to face away from the camera.

    The IMU wants to be on the rotation axis -- an accelerometer offset radially sees
    centripetal and tangential acceleration it cannot tell from gravity -- but the chassis
    spine occupies the axis at every station. So an off-the-shelf breakout lies flat beside
    the waist, running along it, chip side facing the rod. Centred over the rod, the chip's
    distance from the axis is just the stack-up from the rod surface, wherever the chip sits
    on the board: IMU_RADIAL_OFFSET.
    """
    reach = P.CASING_ID / 2 - 2.0
    t = P.IMU_BRIDGE_THICKNESS
    carrier_x0 = P.IMU_BOARD_FAR_R
    carrier_x1 = carrier_x0 + P.IMU_CARRIER_THICKNESS
    carrier_y = P.IMU_BOARD_WIDTH + 2 * P.IMU_CARRIER_MARGIN
    carrier_z = P.IMU_BOARD_LENGTH + 2 * P.IMU_CARRIER_MARGIN

    carrier = (cq.Workplane("XY")
               .box(carrier_x1 - carrier_x0, carrier_y, carrier_z)
               .translate(((carrier_x0 + carrier_x1) / 2, 0, 0)))
    arm = (cq.Workplane("XY")
           .box(reach - carrier_x1, P.IMU_BRIDGE_WIDTH, t)
           .translate(((carrier_x1 + reach) / 2, 0, 0)))
    bridge = carrier.union(arm)

    # Breakout screws, through the carrier along X.
    hole_z = P.IMU_BOARD_LENGTH / 2 - P.IMU_MOUNT_HOLE_INSET
    for z in (-hole_z, hole_z):
        bridge = bridge.cut(
            cq.Workplane("YZ", origin=(carrier_x0, 0, z))
            .circle(P.IMU_MOUNT_HOLE_DIA / 2).extrude(P.IMU_CARRIER_THICKNESS))

    # Fixing to the casing wall.
    bridge = bridge.cut(
        cq.Workplane("XY", origin=(reach - 5.0, 0, -t / 2))
        .circle(P.M3_CLEARANCE / 2).extrude(t))
    return bridge


def imu_board_envelope() -> cq.Workplane:
    """The breakout plus everything on its chip side, as one box, in imu_bridge's frame.

    Not a printed part. It exists so the sweep can check the board's clearance to the waist,
    which is the gap that sets the IMU's radial offset.
    """
    near, far = P.IMU_BOARD_NEAR_R, P.IMU_BOARD_FAR_R
    return (cq.Workplane("XY")
            .box(far - near, P.IMU_BOARD_WIDTH, P.IMU_BOARD_LENGTH)
            .translate(((near + far) / 2, 0, 0)))


# --------------------------------------------------------------------------- part 6

def camera_mount() -> cq.Workplane:
    """Flat plate carrying the camera module behind the shell aperture.

    Shape: flat plate. Features: four module screw holes, a lens clearance hole. Built
    flat; the assembly stands it up against the wall with its axial side along Z.
    """
    t = P.CAMERA_MOUNT_PLATE_THICKNESS
    plate = (cq.Workplane("XY")
             .rect(P.CAMERA_MOUNT_PLATE_AXIAL, P.CAMERA_MOUNT_PLATE_TANGENTIAL).extrude(t))
    plate = plate.edges("|Z").fillet(3.0)

    holes = [(x, y)
             for x in (-P.CAMERA_MOUNT_HOLE_SPACING_X / 2, P.CAMERA_MOUNT_HOLE_SPACING_X / 2)
             for y in (-P.CAMERA_MOUNT_HOLE_SPACING_Y / 2, P.CAMERA_MOUNT_HOLE_SPACING_Y / 2)]
    plate = (plate.faces(">Z").workplane().pushPoints(holes)
             .circle(P.CAMERA_MOUNT_HOLE_DIA / 2).cutThruAll())

    plate = plate.faces(">Z").workplane().circle(P.CAMERA_LENS_HOLE_DIA / 2).cutThruAll()
    return plate


# --------------------------------------------------------------------------- part 7

def wheel() -> cq.Workplane:
    """Rim, hub and a thin spoke web, with an O-ring tread groove. Two needed.

    Shape: cylinder. The O-ring is the tread (R7 trick): no tread pattern to model, and it
    is replaceable when it wears. Built as a thin rim carrying the tread, a small hub, and a
    recessed web between them, because a near-solid wheel was once close to half the
    vehicle's mass budget.

    WHEEL_OD must exceed CASING_OD or the casing drags; ground clearance is
    (WHEEL_OD - CASING_OD) / 2.
    """
    r_out = P.WHEEL_OD / 2
    width = P.WHEEL_WIDTH
    groove = P.WHEEL_ORING_CROSS_SECTION
    rim_thickness = 4.0
    web_thickness = 3.0
    hub_outer = P.WHEEL_HUB_OD / 2
    spoke_width = 8.0

    rim = cq.Workplane("XY").circle(r_out).circle(r_out - rim_thickness).extrude(width)
    hub = cq.Workplane("XY").circle(hub_outer).extrude(width)

    web_z = (width - web_thickness) / 2
    spokes = cq.Workplane("XY")
    for x, y in _ring_points(1.0, P.WHEEL_SPOKE_COUNT):
        angle = math.degrees(math.atan2(y, x))
        spokes = spokes.union(
            cq.Workplane("XY", origin=(0, 0, web_z))
            .rect(r_out * 2, spoke_width).extrude(web_thickness)
            .rotate((0, 0, 0), (0, 0, 1), angle))
    spokes = spokes.intersect(
        cq.Workplane("XY", origin=(0, 0, web_z))
        .circle(r_out - rim_thickness).extrude(web_thickness))

    body = rim.union(hub).union(spokes)

    body = body.cut(
        cq.Workplane("XY", origin=(0, 0, (width - groove) / 2))
        .circle(r_out).circle(r_out - groove * 0.6)
        .extrude(groove))

    body = body.faces(">Z").workplane().circle(P.WHEEL_BORE / 2).cutThruAll()
    return body


ALL_PARTS = {
    "01_casing_shell": casing_shell,
    "02_casing_end_cap_a": casing_end_cap_a,
    "03_casing_end_cap_b": casing_end_cap_b,
    "04_chassis_spine": chassis_spine,
    "05_imu_bridge": imu_bridge,
    "06_camera_mount": camera_mount,
    "07_wheel": wheel,
}
