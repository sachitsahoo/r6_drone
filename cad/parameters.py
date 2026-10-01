"""Every dimension for the Recon UGV mechanical parts, in millimetres.

Single source of truth for the CAD. Nothing in parts.py contains a literal dimension.

Each value is tagged with where it came from, because the difference matters:

  OWNER      given directly by the owner
  DERIVED    computed from OWNER values or from the analysis in docs/theory/
  ASSUMPTION **invented here because the design needs a number.** These are the ones to
             review and change; none of them is backed by analysis.

Cross-references: docs/mechanical-requirements.md (R1-R8),
docs/decisions/0008-reduced-scale-direct-drive.md (scale and direct drive),
docs/decisions/0010-direct-drive-axial-layout.md (why the chassis is a spine).
"""

from __future__ import annotations

import math

# ---------------------------------------------------------------------------- casing

CASING_OD = 70.0             # OWNER: ADR 0008, chosen for inertia (I scales ~ r^4 at
                             # fixed proportions)
CASING_WALL = 2.5            # OWNER: range 2.5-3, thin end chosen (less inertia)
CASING_LENGTH = 182.0        # OWNER: ADR 0008, length / wheel diameter ~2.0
CASING_ID = CASING_OD - 2 * CASING_WALL          # DERIVED: 65.0

#: The shell's end walls are too thin for axial tapped holes, so each end carries an
#: internal flange: this deep axially and this far radially inward.
SHELL_FLANGE_DEPTH = 4.0     # ASSUMPTION: enough thread for an M3 tap
SHELL_FLANGE_RADIAL = 4.5    # ASSUMPTION: wide enough to land an M3 tap with ~1 mm each side

# ---------------------------------------------------------------- chassis (spine)
#
# ADR 0010: the chassis is a single spine on the axis, not two discs joined by standoffs.
# The casing turns all the way round relative to the chassis, so every chassis feature
# sweeps a full ring and casing contents can only live outside the chassis's largest
# radius at each axial station. Standoffs at r=35 (the old design) would have claimed the
# whole interior. A spine claims only its own radius.

SPINE_OD = 16.0              # ASSUMPTION: 12 mm N20 body plus a 2 mm printed wall
SPINE_BORE = 13.0            # ASSUMPTION: clears an N20 motor body (12 mm) by 0.5 each side
SPINE_END_WALL = 2.0         # ASSUMPTION: closes each wheel motor pocket on its inboard end
#: The thinned section the IMU sits beside. In reality a bought rod joining two printed
#: ends -- a printed 5 mm section would not survive a throw. A hollow tube lets wheel motor
#: A's wires pass through it; a threaded rod lets nuts clamp the two ends together. Its
#: radius sets the IMU's radial offset, so it is kept as small as is credible.
SPINE_WAIST_OD = 5.0         # ASSUMPTION: M5 threaded rod or 5 mm brass/aluminium tube;
                             # the smallest that plausibly carries wheel loads

# Wheel motors live inside the spine, one at each end, shafts pointing outboard.
WHEEL_MOTOR_OD = 12.0        # LISTING: N20 gearmotor body
WHEEL_MOTOR_LENGTH = 40.0    # GUESS: depends entirely on gear ratio -- measure

# ------------------------------------------------------------- pitch motor (direct)
#
# ADR 0008: the motor drives the casing directly, coaxial with the wheel axis. Its stator
# bolts to the chassis cup at end A and its rotor bell bolts to end cap A.
#
# Wheel A's drive shaft has to pass through the motor's centre -- there is nowhere else
# for it to go (ADR 0010). So this must be a HOLLOW-SHAFT motor.

# iPower GM2804 (ADR 0011). iFlight listing, 2026-09-30.
#
# Orientation: the STATOR bolts to end cap A (casing) and the ROTOR to the cup floor
# (chassis). The windings are on the stator, so this keeps the three phase leads inside the
# casing with the power stage -- the reason R3 exists. Bolted the other way, the phase leads
# would have to cross the wire loop. Chosen because it is a real,
# stocked hollow-shaft gimbal motor: a 28 mm 2208-class part with a bore that passes a shaft
# could not be found. 35 mm is its OD despite the "28" in the name (that is the stator).
PITCH_MOTOR_OD = 35.0                   # LISTING: "Phi 35 +/- 0.05 mm"
#: Axial length. The listing gives 25 mm for the encoder variant "with encoder housing";
#: the plain motor is assumed no longer. LISTING
PITCH_MOTOR_LENGTH = 25.0
PITCH_MOTOR_MASS_G = 51.0               # LISTING (encoder variant)
#: "Hollow shaft ID Phi 6.5 +0.05/0". The GBM2804H variant lists 5 mm -- buy the GM2804H.
PITCH_MOTOR_HOLLOW_BORE = 6.5           # LISTING
PITCH_MOTOR_POLE_PAIRS = 7              # LISTING: 12N14P
#: Rotor-bell bolt circle, into the cup floor. Must clear the floor's 13 mm bore (wheel
#: motor A goes in through it), which the test suite checks. ASSUMPTION: measure.
PITCH_MOTOR_ROTOR_BOLT_RADIUS = 8.0
#: Stator base bolt circle, into end cap A. M2, per the listing's Q&A. ASSUMPTION: measure.
PITCH_MOTOR_STATOR_BOLT_RADIUS = 9.5
PITCH_MOTOR_BOLT_COUNT = 4

#: Radial gap between the motor body and the cup wall around it.
MOTOR_CUP_RADIAL_CLEARANCE = 1.0        # ASSUMPTION: FDM bores run undersize
CUP_WALL = 1.5                          # ASSUMPTION: thin, but it only locates a bearing race
CUP_FLOOR_THICKNESS = 3.0               # ASSUMPTION: carries the stator bolts

# Wheel A's shaft: N20 output extended through the pitch motor's bore to the wheel.
WHEEL_SHAFT_DIA = 3.0        # LISTING: N20 D-shaft
#: The N20's own shaft is ~10 mm, so wheel A needs an extension of ~55 mm: a coupler plus
#: a 3 mm rod. Wheel B, at the other end, reaches its wheel directly.

# ------------------------------------------------------------------- bearings
#
# End A: the casing bearing sits on the OUTSIDE of the cup wall, so impact loads on the
# casing go to the chassis through a real bearing rather than through the gimbal motor's
# tiny internal ones.
BEARING_A_ID = 40.0          # LISTING: 6708ZZ thin-section, 40 x 50 x 6 (simplybearings)
BEARING_A_OD = 50.0
BEARING_A_WIDTH = 6.0
# End B: unchanged from the first design, on a boss at the spine's end.
AXIS_BOSS_OD = 20.0          # ASSUMPTION: bearing bore rides on this
BEARING_OD = 27.0            # ASSUMPTION: 6704ZZ, 20 x 27 x 4
BEARING_ID = 20.0
BEARING_WIDTH = 4.0

#: Smallest running gap anywhere between parts that move relative to each other.
RUNNING_CLEARANCE = 0.5      # ASSUMPTION: same as the first design's cap-to-boss gap

# ----------------------------------------------------------------------------- wheels

WHEEL_OD = 105.0             # OWNER: ADR 0008. Ground clearance (105 - 70) / 2 = 17.5 mm,
                             # the same as the 170 mm wheel gave the 135 mm casing
#: Overall width is 214 mm (ADR 0008), so each side gets (214 - 182) / 2 = 16 mm for the
#: wheel plus the gap to the casing.
OVERALL_WIDTH = 214.0        # OWNER: ADR 0008
WHEEL_STANDOFF = 4.0         # ASSUMPTION: gap between wheel and casing end
WHEEL_WIDTH = (OVERALL_WIDTH - CASING_LENGTH) / 2 - WHEEL_STANDOFF   # DERIVED: 12.0
WHEEL_BORE = WHEEL_SHAFT_DIA # DERIVED
WHEEL_ORING_CROSS_SECTION = 3.0   # ASSUMPTION: O-ring tread (R7 trick)
WHEEL_HUB_OD = 16.0          # ASSUMPTION
WHEEL_SPOKE_COUNT = 5        # ASSUMPTION: cosmetic and mass reduction

# ------------------------------------------------------------------------------ camera

CAMERA_APERTURE_WIDTH = 18.0     # ASSUMPTION: clears the Camera Module 3 Wide lens barrel
CAMERA_APERTURE_HEIGHT = 14.0    # ASSUMPTION
CAMERA_MOUNT_HOLE_SPACING_X = 21.0   # ASSUMPTION: confirm against the module drawing
CAMERA_MOUNT_HOLE_SPACING_Y = 12.5   # ASSUMPTION
CAMERA_MOUNT_HOLE_DIA = 2.2      # ASSUMPTION: M2 clearance
CAMERA_MOUNT_PLATE_THICKNESS = 3.0
#: Along the axis, and tangential, once mounted. The tangential one is what limits how
#: far out the plate can sit: its corners are what reach the wall first.
CAMERA_MOUNT_PLATE_AXIAL = 30.0      # ASSUMPTION
CAMERA_MOUNT_PLATE_TANGENTIAL = 26.0 # ASSUMPTION: Camera Module 3 board is 25 x 24
CAMERA_LENS_HOLE_DIA = 10.0          # ASSUMPTION

# --------------------------------------------------------------------------------- IMU

# R2 (amended by ADR 0010): the IMU cannot sit ON the axis, because the chassis passes every
# axial station (see the spine note above). An off-the-shelf breakout lies flat beside the
# spine's waist instead, running along it, CHIP SIDE FACING THE ROD. Centred over the rod,
# the chip's distance from the axis is just the stack-up below, wherever the chip sits on the
# board -- so no custom PCB is needed (custom PCBs are out of scope, CLAUDE.md).
IMU_BOARD_LENGTH = 20.0          # GUESS: breakout outline, along the axis once mounted
IMU_BOARD_WIDTH = 16.0           # GUESS: tangential once mounted
IMU_BOARD_THICKNESS = 1.6        # GUESS: standard FR-4
#: Tallest thing on the chip side of the breakout. If a capacitor or regulator is taller
#: than the IMU package, it sets the gap, not the chip. Headers must go on the back.
IMU_CHIP_SIDE_HEIGHT = 1.0       # GUESS: measure the real breakout
IMU_PACKAGE_THICKNESS = 0.91     # LISTING: ICM-42688-P LGA, TDK datasheet package outline
IMU_WAIST_CLEARANCE = 0.75       # ASSUMPTION: running gap between board and spine waist
IMU_CARRIER_THICKNESS = 3.0      # ASSUMPTION: printed plate the breakout screws to
IMU_CARRIER_MARGIN = 1.0         # ASSUMPTION: carrier overhang past the board on each side
IMU_BRIDGE_THICKNESS = 3.0
IMU_BRIDGE_WIDTH = 12.0
IMU_MOUNT_HOLE_DIA = 2.2         # ASSUMPTION: M2 clearance
IMU_MOUNT_HOLE_INSET = 2.5       # GUESS: breakout mounting holes from the board's ends
#: Radius of the board's chip-side face (its components' tips) and of the chip's centre.
IMU_BOARD_NEAR_R = SPINE_WAIST_OD / 2 + IMU_WAIST_CLEARANCE                      # DERIVED: 3.25
IMU_RADIAL_OFFSET = IMU_BOARD_NEAR_R + IMU_CHIP_SIDE_HEIGHT - IMU_PACKAGE_THICKNESS / 2
                                                                                 # DERIVED: 3.8
IMU_BOARD_FAR_R = IMU_BOARD_NEAR_R + IMU_CHIP_SIDE_HEIGHT + IMU_BOARD_THICKNESS  # DERIVED

# ------------------------------------------------------------------- fasteners, fixtures

M3_CLEARANCE = 3.4
M3_TAP = 2.5
M2_CLEARANCE = 2.2

TRIM_BOSS_COUNT = 6             # R4
TRIM_BOSS_OD = 7.0
TRIM_BOSS_HEIGHT = 5.0

PENDULUM_DATUM_RADIUS = 27.5    # R8: known offset for the inertia measurement, on cap B's rim
PENDULUM_DATUM_ANGLE_DEG = 30.0 # DERIVED: midway between two cap screws
PENDULUM_DATUM_DIA = 3.2


# End cap B: rim, hub and spokes, stepped in thickness. 6 mm is needed only at the hub,
# for a 4 mm bearing seat plus a 2 mm shoulder.
END_CAP_THICKNESS = 6.0
END_CAP_RIM_THICKNESS = 4.0
#: Clearance bore through the cap's shoulder, so it passes the boss without rubbing.
END_CAP_BOSS_CLEARANCE_BORE = AXIS_BOSS_OD + 2 * RUNNING_CLEARANCE   # DERIVED: 21.0
END_CAP_SCREW_COUNT = 6
END_CAP_SCREW_RADIUS = CASING_ID / 2 - SHELL_FLANGE_RADIAL / 2      # DERIVED: mid-flange

# End cap A (motor end): thicker, because it holds the bearing seat around the cup wall
# with a shoulder under it, and the motor's stator bolts to its inboard face.
END_CAP_A_THICKNESS = 9.0       # ASSUMPTION: 6.5 mm seat + 2.5 mm shoulder
#: Annulus outside the bearing housing: carries only screw loads, so it runs thinner.
END_CAP_A_HOUSING_WALL = 3.0    # ASSUMPTION: material around the bearing's outer race
END_CAP_A_SHAFT_BORE = PITCH_MOTOR_HOLLOW_BORE                       # DERIVED

# --------------------------------------------------------------------------- materials

# For mass and inertia reporting only.
PLA_DENSITY = 1.24e-3           # g/mm^3
PETG_DENSITY = 1.27e-3
ABS_DENSITY = 1.04e-3
PRINT_DENSITY = ABS_DENSITY     # what the reports assume; ABS is 18% lighter than
                                # PETG, which is a free lever on rotating inertia


# ------------------------------------------- derived cup geometry (do not edit)

#: The cup wall's OD is the end-A bearing's bore; its ID clears the motor.
CUP_OD = BEARING_A_ID                    # 40
CUP_ID = CUP_OD - 2 * CUP_WALL           # 37: the 35 mm motor plus 1 mm each side
assert CUP_ID >= PITCH_MOTOR_OD + 2 * MOTOR_CUP_RADIAL_CLEARANCE - 1e-9, \
    "the cup wall would rub the motor body"

#: Groove in end cap A's inboard face: the bearing's outer race at its outer edge, the cup
#: wall plus a running gap at its inner edge, and half a millimetre deeper than the bearing
#: so the cup tip does not touch the groove floor.
END_CAP_A_GROOVE_INNER_R = CUP_ID / 2 - RUNNING_CLEARANCE
END_CAP_A_GROOVE_OUTER_R = BEARING_A_OD / 2
END_CAP_A_GROOVE_DEPTH = BEARING_A_WIDTH + RUNNING_CLEARANCE

# --------------------------------------- axial layout, z from casing face A (do not edit)
#
# Every station below follows from the parts' own lengths. The first design's layout was a
# hand-placed PROPOSAL and collided with itself four times as parts changed; deriving it
# removes that class of bug. End A is the motor end.

Z_END_CAP_A = SHELL_FLANGE_DEPTH                         # cap A seats on the flange: 4
Z_MOTOR_LO = Z_END_CAP_A + END_CAP_A_THICKNESS           # stator base on cap A: 13
Z_MOTOR_HI = Z_MOTOR_LO + PITCH_MOTOR_LENGTH             # rotor bell on the floor: 38
Z_CUP_TIP = Z_MOTOR_LO - BEARING_A_WIDTH                 # cup wall fills the bearing: 7
Z_CUP_FLOOR_HI = Z_MOTOR_HI + CUP_FLOOR_THICKNESS        # 41
Z_WHEEL_MOTOR_A_LO = Z_CUP_FLOOR_HI                      # 41
Z_WHEEL_MOTOR_A_HI = Z_WHEEL_MOTOR_A_LO + WHEEL_MOTOR_LENGTH        # 81
Z_WAIST_LO = Z_WHEEL_MOTOR_A_HI + SPINE_END_WALL         # 83

Z_SPINE_END = CASING_LENGTH                              # boss B flush with the shell: 182
Z_WHEEL_MOTOR_B_HI = Z_SPINE_END                         # shaft exits the boss end
Z_WHEEL_MOTOR_B_LO = Z_WHEEL_MOTOR_B_HI - WHEEL_MOTOR_LENGTH        # 142
Z_WAIST_HI = Z_WHEEL_MOTOR_B_LO - SPINE_END_WALL         # 140
Z_END_CAP_B = CASING_LENGTH - SHELL_FLANGE_DEPTH - END_CAP_THICKNESS   # 172
#: Boss B reaches inboard of cap B's bearing seat by this much, so it is a boss and not a
#: lip. ASSUMPTION.
AXIS_BOSS_INBOARD_OVERHANG = 6.0
Z_BOSS_B_LO = Z_END_CAP_B - AXIS_BOSS_INBOARD_OVERHANG   # 166

Z_CAMERA = CASING_LENGTH / 2                             # 91
#: Axial centre of the IMU board, on the waist and clear of the camera mount's span.
#: ASSUMPTION within those limits; the test suite checks both.
Z_IMU_BRIDGE = 120.0
#: Trim bosses sit at the casing's balance station, away from the camera (which shares the
#: 0-degree angle with one boss). ASSUMPTION: anywhere axially over the thick spine works.
Z_TRIM_BOSSES = 55.0

Z_WHEEL_A = -WHEEL_WIDTH - WHEEL_STANDOFF                # -16
Z_WHEEL_B = CASING_LENGTH + WHEEL_STANDOFF               # 186

#: Radial position of the camera mount's inner face. Its corners are the first thing to
#: reach the wall, so solve for the corner touching the bore minus a running gap.
CAMERA_MOUNT_INNER_RADIUS = (
    math.sqrt((CASING_ID / 2 - RUNNING_CLEARANCE) ** 2 - (CAMERA_MOUNT_PLATE_TANGENTIAL / 2) ** 2)
    - CAMERA_MOUNT_PLATE_THICKNESS)
