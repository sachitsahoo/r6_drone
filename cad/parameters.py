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
#: The thinned section the IMU ring sits around. In reality a bought 5 mm steel rod or
#: carbon tube plugged into two printed ends -- a printed 5 mm section would not survive a
#: throw. Its radius sets the IMU's radial offset, so it is kept as small as is credible.
SPINE_WAIST_OD = 5.0         # ASSUMPTION: smallest rod that plausibly carries wheel loads

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

# 28 mm matches a 2208-class gimbal motor (vendor listings: 28 mm OD, 39-42 g). The more
# commonly recommended GM2804 is 35 mm OD despite its name, and would force a larger cup.
PITCH_MOTOR_OD = 28.0
#: Axial length of the motor body. SpeedyFPV 2208 listing: 26 mm.
PITCH_MOTOR_LENGTH = 26.0
PITCH_MOTOR_MASS_G = 39.0
PITCH_MOTOR_KV = 80.0
#: ASSUMPTION, and the one to check before buying anything: hollow-shaft gimbal motors exist
#: but whether a 28 mm one with a >= 5 mm bore is easy to source has NOT been verified.
PITCH_MOTOR_HOLLOW_BORE = 5.0
PITCH_MOTOR_ROTOR_BOLT_RADIUS = 6.0     # ASSUMPTION: rotor-bell top bolt circle
PITCH_MOTOR_STATOR_BOLT_RADIUS = 9.5    # ASSUMPTION: stator base bolt circle
PITCH_MOTOR_BOLT_COUNT = 4

#: Radial gap between the motor body and the cup wall around it.
MOTOR_CUP_RADIAL_CLEARANCE = 1.5        # ASSUMPTION: FDM bores run undersize
CUP_WALL = 2.0                          # ASSUMPTION
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
BEARING_A_ID = 35.0          # ASSUMPTION: 61807 thin-section, 35 x 44 x 5
BEARING_A_OD = 44.0
BEARING_A_WIDTH = 5.0
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
# axial station (see the spine note above). It sits on a pad with a hole around the spine's
# waist instead, and the requirement is "under 5 mm radially, error budgeted".
IMU_PAD_SIZE = 16.0
IMU_BRIDGE_THICKNESS = 3.0
IMU_BRIDGE_WIDTH = 12.0
IMU_MOUNT_HOLE_DIA = 2.2         # ASSUMPTION: M2 clearance
IMU_WAIST_CLEARANCE = 0.75       # ASSUMPTION: radial gap between pad hole and spine waist
#: Half the ICM-42688-P package (2.5 x 3 mm LGA). The sensing element is near its centre.
IMU_PACKAGE_HALF = 1.5           # LISTING: TDK datasheet package outline
IMU_PAD_HOLE_DIA = SPINE_WAIST_OD + 2 * IMU_WAIST_CLEARANCE          # DERIVED: 6.5
#: Where the sensor actually sits, if it is placed right at the hole's edge. This needs a
#: custom ring breakout -- off-the-shelf breakouts put the chip mid-board.
IMU_RADIAL_OFFSET = IMU_PAD_HOLE_DIA / 2 + IMU_PACKAGE_HALF          # DERIVED: 4.75

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

SLIP_RING_ENVELOPE_DIA = 15.0   # R5. Fallback only: ADR 0009 proposes a wire loop instead
SLIP_RING_ENVELOPE_LENGTH = 25.0

# End cap B: rim, hub and spokes, stepped in thickness. 6 mm is needed only at the hub,
# for a 4 mm bearing seat plus a 2 mm shoulder.
END_CAP_THICKNESS = 6.0
END_CAP_RIM_THICKNESS = 4.0
#: Clearance bore through the cap's shoulder, so it passes the boss without rubbing.
END_CAP_BOSS_CLEARANCE_BORE = AXIS_BOSS_OD + 2 * RUNNING_CLEARANCE   # DERIVED: 21.0
END_CAP_SCREW_COUNT = 6
END_CAP_SCREW_RADIUS = CASING_ID / 2 - SHELL_FLANGE_RADIAL / 2      # DERIVED: mid-flange

# End cap A (motor end): thicker, because it holds a 5 mm bearing seat around the cup wall
# with a shoulder under it, and the motor's rotor bolts to its inboard face.
END_CAP_A_THICKNESS = 8.0       # ASSUMPTION: 5.5 mm seat + 2.5 mm shoulder
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
CUP_OD = BEARING_A_ID
CUP_ID = CUP_OD - 2 * CUP_WALL
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
Z_MOTOR_LO = Z_END_CAP_A + END_CAP_A_THICKNESS           # rotor bell on cap A: 12
Z_MOTOR_HI = Z_MOTOR_LO + PITCH_MOTOR_LENGTH             # stator base: 38
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
#: The IMU sits on the waist, clear of the camera mount's axial span. ASSUMPTION within
#: those limits; the test suite checks both.
Z_IMU_BRIDGE = 118.0
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
