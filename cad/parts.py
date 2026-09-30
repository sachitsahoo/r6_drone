"""Parametric solids for the Recon UGV mechanical parts.

Every dimension comes from parameters.py; there are no literals here beyond counts and
trivial fractions. Eight parts, built from four shape types -- tube, disc, flat plate,
cylinder -- exactly as docs/mechanical-requirements.md describes.

This is a STARTING POINT, not a finished design. Several dimensions in parameters.py are
tagged ASSUMPTION because the design needed a number and no analysis supplies one; the
wheel hub interface in particular is a placeholder. Read that file before printing anything.

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


# --------------------------------------------------------------------------- part 1

def casing_shell() -> cq.Workplane:
    """The rotating outer casing: a tube, plus an aperture and mounting features.

    Shape: cylinder minus cylinder. Features: camera aperture, internal end flanges for
    the end cap screws, trim mass bosses (R4), and the pendulum datum hole (R8).
    """
    r_out = P.CASING_OD / 2
    r_in = P.CASING_ID / 2
    length = P.CASING_LENGTH

    shell = cq.Workplane("XY").circle(r_out).circle(r_in).extrude(length)

    # Internal flanges at both ends: the 3 mm wall is too thin for axial tapped holes, so
    # thicken it locally rather than thickening the whole shell (which would cost inertia
    # at the largest radius -- see R6).
    flange_depth = 4.0
    flange_r_in = r_in - 4.5
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

    # Camera aperture, cut radially through the wall at mid-length.
    aperture = (cq.Workplane("YZ", origin=(0, 0, length / 2))
                .rect(P.CAMERA_APERTURE_WIDTH, P.CAMERA_APERTURE_HEIGHT)
                .extrude(r_out + 1.0))
    shell = shell.cut(aperture)

    # Trim mass bosses (R4): radial pads on the inner wall with tapped holes, so the
    # balanced-vs-bottom-heavy decision stays open after assembly.
    # A YZ workplane extrudes along +X from wherever its origin sits, so starting at x=0
    # and extruding by r_in produced a solid ROD from the rotation axis out to the wall --
    # six of them, 5530 mm^3 of material in what is supposed to be a hollow tube, with the
    # tap drilled clean through the impact surface. Start the plane just inside the wall
    # instead, and keep the tap blind so nothing pierces the shell.
    boss_z = length / 2
    boss_x0 = r_in - P.TRIM_BOSS_HEIGHT
    for x, y in _ring_points(1.0, P.TRIM_BOSS_COUNT):          # unit vector per position
        angle = math.degrees(math.atan2(y, x))
        pad = (cq.Workplane("YZ", origin=(boss_x0, 0, boss_z))
               .circle(P.TRIM_BOSS_OD / 2)
               .extrude(P.TRIM_BOSS_HEIGHT)
               .rotate((0, 0, 0), (0, 0, 1), angle))
        tap = (cq.Workplane("YZ", origin=(boss_x0, 0, boss_z))
               .circle(P.M3_TAP / 2)
               .extrude(P.TRIM_BOSS_HEIGHT - 1.0)      # blind: does not reach the wall
               .rotate((0, 0, 0), (0, 0, 1), angle))
        shell = shell.union(pad).cut(tap)

    return shell


# --------------------------------------------------------------------------- part 2

def casing_end_cap(with_datum: bool = False) -> cq.Workplane:
    """Rim, hub and spokes, stepped in thickness. Two needed.

    Shape: disc. Features: bearing seat in the hub, screw holes matching the shell's
    flanges, and on one cap the pendulum datum hole (R8).

    Built as rim + hub + spokes because a solid 130 mm disc weighs 63 g, and it sits at the
    largest radius in the rotating assembly -- the worst place in the machine to put mass.

    Stepped because the 6 mm thickness is needed *only* at the hub, where the bearing needs
    a 4 mm seat plus a shoulder. The rim and spokes carry screw loads and nothing else, so
    they run at 4 mm. Carrying 6 mm out to r=65 cost about 7 g per cap at maximum radius.
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
    # Clearance bore for the chassis boss, through the remaining shoulder.
    cap = cap.cut(
        cq.Workplane("XY").circle(P.END_CAP_BOSS_CLEARANCE_BORE / 2).extrude(hub_t))

    cap = cap.cut(
        cq.Workplane("XY")
        .pushPoints(_ring_points(P.END_CAP_SCREW_RADIUS, P.END_CAP_SCREW_COUNT))
        .circle(P.M3_CLEARANCE / 2).extrude(rim_t))

    if with_datum:
        cap = cap.cut(
            cq.Workplane("XY")
            .pushPoints([(P.PENDULUM_DATUM_RADIUS, 0.0)])
            .circle(P.PENDULUM_DATUM_DIA / 2).extrude(rim_t))
    return cap


# --------------------------------------------------------------------------- part 3

def chassis_disc() -> cq.Workplane:
    """Rim, hub and spokes, with a central boss the casing bearing rides on. Two needed.

    Shape: disc. Spokes are aligned with the standoff positions so the fixing holes land in
    material. Mass here is off the rotating assembly, so it costs vehicle mass rather than
    inertia -- but the first build had two of these at 111 g, which the budget cannot afford.
    """
    r_out = P.CHASSIS_OD / 2
    thickness = P.CHASSIS_DISC_THICKNESS
    rim_inner = r_out - 6.0
    hub_outer = P.AXIS_BOSS_OD / 2 + 4.0
    spoke_width = 13.0

    rim = cq.Workplane("XY").circle(r_out).circle(rim_inner).extrude(thickness)
    hub = cq.Workplane("XY").circle(hub_outer).extrude(thickness)

    spokes = cq.Workplane("XY")
    for x, y in _ring_points(1.0, P.STANDOFF_COUNT):
        angle = math.degrees(math.atan2(y, x))
        spokes = spokes.union(
            cq.Workplane("XY").rect(r_out * 2, spoke_width).extrude(thickness)
            .rotate((0, 0, 0), (0, 0, 1), angle))
    spokes = spokes.intersect(cq.Workplane("XY").circle(rim_inner).extrude(thickness))

    disc = rim.union(hub).union(spokes)

    boss = (cq.Workplane("XY", origin=(0, 0, thickness))
            .circle(P.AXIS_BOSS_OD / 2).extrude(P.AXIS_BOSS_LENGTH))
    disc = disc.union(boss)
    disc = disc.faces(">Z").workplane().circle(P.AXIS_BOSS_BORE / 2).cutThruAll()

    disc = (disc.faces("<Z").workplane()
            .pushPoints(_ring_points(P.STANDOFF_RADIUS, P.STANDOFF_COUNT))
            .circle(P.M3_CLEARANCE / 2).cutThruAll())
    return disc


# --------------------------------------------------------------------------- part 4

def chassis_drive_band() -> cq.Workplane:
    """Short tube at the reduced drive diameter; the GT2 belt bonds onto its outside (R1).

    Shape: short tube. No teeth: a length of timing belt bonded teeth-outward *is* the
    toothed rack, which removes the only difficult geometry in the design.
    """
    r_out = P.DRIVE_BAND_OD / 2
    width = P.DRIVE_BAND_WIDTH

    band = (cq.Workplane("XY")
            .circle(r_out).circle(P.CHASSIS_FRAME_OD / 2)
            .extrude(width))

    # Shallow channel to locate the bonded belt axially and stop it walking off.
    channel_depth = 0.6
    band = band.cut(
        cq.Workplane("XY", origin=(0, 0, (width - P.BELT_WIDTH) / 2))
        .circle(r_out).circle(r_out - channel_depth)
        .extrude(P.BELT_WIDTH))

    band = (band.faces(">Z").workplane()
            .pushPoints(_ring_points(P.STANDOFF_RADIUS, P.STANDOFF_COUNT))
            .circle(P.M3_CLEARANCE / 2).cutThruAll())

    return band


# --------------------------------------------------------------------------- part 5

def pitch_motor_bracket() -> cq.Workplane:
    """Flat plate holding the pitch motor in the casing, pulley reaching the drive band.

    Shape: flat plate. Features: motor bore, motor bolt circle, mounting slots for belt
    tension adjustment.
    """
    plate_l = P.MOTOR_BRACKET_RADIAL_SPAN
    plate_w = P.MOTOR_BRACKET_TANGENTIAL_SPAN
    t = P.MOTOR_BRACKET_THICKNESS

    plate = cq.Workplane("XY").rect(plate_l, plate_w).extrude(t)
    plate = plate.edges("|Z").fillet(4.0)

    plate = plate.faces(">Z").workplane().circle(P.PITCH_MOTOR_BORE / 2).cutThruAll()
    plate = (plate.faces(">Z").workplane()
             .pushPoints(_ring_points(P.PITCH_MOTOR_BOLT_RADIUS, P.PITCH_MOTOR_BOLT_COUNT,
                                      phase_deg=45.0))
             .circle(P.M2_CLEARANCE / 2).cutThruAll())

    # Trim to the casing's inner curve and the chassis frame's outer curve. A plain
    # rectangle at this radius has CORNERS that poke through the shell even when its flat
    # faces clear it -- the first assembly check caught exactly that. The part stays
    # centred on its own origin; the trim cylinders are offset by the pulley radius.
    outer_trim = (cq.Workplane("XY", origin=(-P.DRIVE_PULLEY_CENTER_RADIUS, 0, 0))
                  .circle(P.CASING_ID / 2 - 1.0).extrude(t))
    plate = plate.intersect(outer_trim)

    inner_trim = (cq.Workplane("XY", origin=(-P.DRIVE_PULLEY_CENTER_RADIUS, 0, 0))
                  .circle(P.CHASSIS_FRAME_OD / 2 + 1.0).extrude(t))
    plate = plate.cut(inner_trim)

    # Slots, not holes: belt tension needs adjustment (ADR 0006).
    for x in (-plate_l / 2 + 4.0, plate_l / 2 - 4.0):
        slot = (cq.Workplane("XY", origin=(x, 0, 0))
                .slot2D(10.0, P.M3_CLEARANCE, 90.0).extrude(t))
        plate = plate.cut(slot)

    return plate


# --------------------------------------------------------------------------- part 6

def imu_bridge() -> cq.Workplane:
    """Flat strip reaching from the casing wall to the rotation centreline (R2).

    Shape: flat strip. The IMU must sit within ~3 mm radially of the axis: a 30 mm offset
    corrupts the gravity reference by 17 degrees at only 10 rad/s. The constraint is radial
    only, so this reaches the centreline at an axial station clear of the hub.
    """
    reach = P.CASING_ID / 2 - 2.0
    t = P.IMU_BRIDGE_THICKNESS

    arm = (cq.Workplane("XY")
           .moveTo(0, -P.IMU_BRIDGE_WIDTH / 2)
           .lineTo(reach, -P.IMU_BRIDGE_WIDTH / 2)
           .lineTo(reach, P.IMU_BRIDGE_WIDTH / 2)
           .lineTo(0, P.IMU_BRIDGE_WIDTH / 2)
           .close().extrude(t))

    pad = (cq.Workplane("XY")
           .rect(P.IMU_PAD_SIZE, P.IMU_PAD_SIZE).extrude(t))
    bridge = arm.union(pad)

    # IMU screws straddling the axis, so the sensor body sits centred on it.
    offset = P.IMU_PAD_SIZE / 2 - 3.0
    bridge = (bridge.faces(">Z").workplane()
              .pushPoints([(-offset, -offset), (offset, offset)])
              .circle(P.IMU_MOUNT_HOLE_DIA / 2).cutThruAll())

    # Fixing to the casing wall.
    bridge = (bridge.faces(">Z").workplane()
              .pushPoints([(reach - 5.0, 0.0)])
              .circle(P.M3_CLEARANCE / 2).cutThruAll())

    return bridge


# --------------------------------------------------------------------------- part 7

def camera_mount() -> cq.Workplane:
    """Flat plate carrying the camera module behind the shell aperture.

    Shape: flat plate. Features: four module screw holes, a lens clearance hole.
    """
    t = P.CAMERA_MOUNT_PLATE_THICKNESS
    plate = cq.Workplane("XY").rect(30.0, 26.0).extrude(t)
    plate = plate.edges("|Z").fillet(3.0)

    holes = [(x, y)
             for x in (-P.CAMERA_MOUNT_HOLE_SPACING_X / 2, P.CAMERA_MOUNT_HOLE_SPACING_X / 2)
             for y in (-P.CAMERA_MOUNT_HOLE_SPACING_Y / 2, P.CAMERA_MOUNT_HOLE_SPACING_Y / 2)]
    plate = (plate.faces(">Z").workplane().pushPoints(holes)
             .circle(P.CAMERA_MOUNT_HOLE_DIA / 2).cutThruAll())

    plate = plate.faces(">Z").workplane().circle(5.0).cutThruAll()
    return plate


# --------------------------------------------------------------------------- part 8

def wheel() -> cq.Workplane:
    """Rim, hub and a thin spoke web, with an O-ring tread groove. Two needed.

    Shape: cylinder. The O-ring is the tread (R7 trick): no tread pattern to model, and it
    is replaceable when it wears.

    A near-solid 150 mm wheel weighs 160 g, and the first mass build had two of them at
    321 g -- by itself close to half the entire vehicle budget. Built as a thin rim carrying
    the tread, a small hub, and a recessed web between them.

    NOTE: WHEEL_OD is an ASSUMPTION. It must exceed CASING_OD or the casing drags; ground
    clearance is (WHEEL_OD - CASING_OD) / 2, currently 7.5 mm.
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
    "02_casing_end_cap": lambda: casing_end_cap(with_datum=False),
    "02b_casing_end_cap_with_datum": lambda: casing_end_cap(with_datum=True),
    "03_chassis_disc": chassis_disc,
    "04_chassis_drive_band": chassis_drive_band,
    "05_pitch_motor_bracket": pitch_motor_bracket,
    "06_imu_bridge": imu_bridge,
    "07_camera_mount": camera_mount,
    "08_wheel": wheel,
}
