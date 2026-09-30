"""Every dimension for the Recon UGV mechanical parts, in millimetres.

Single source of truth for the CAD. Nothing in parts.py contains a literal dimension.

Each value is tagged with where it came from, because the difference matters:

  OWNER      given directly by the owner
  DERIVED    computed from OWNER values or from the analysis in docs/theory/
  ASSUMPTION **invented here because the design needs a number.** These are the ones to
             review and change; none of them is backed by analysis.

Cross-references: docs/mechanical-requirements.md (R1-R8),
docs/decisions/0004-pitch-axis-architecture.md, docs/decisions/0006-pitch-actuator.md.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------- casing

CASING_OD = 135.0            # OWNER: range 135-145, bottom chosen (less inertia)
CASING_WALL = 2.5            # OWNER: range 2.5-3, thin end chosen after the
                             # first mass build came out 855 g of plastic alone
CASING_LENGTH = 120.0        # DERIVED: axial budget, docs/mechanical-requirements.md
CASING_ID = CASING_OD - 2 * CASING_WALL          # DERIVED: 129.0

ROTATIONAL_CLEARANCE = 2.5   # OWNER: range 2-3

# ---------------------------------------------------------------------------- chassis

CHASSIS_OD = CASING_ID - 2 * ROTATIONAL_CLEARANCE   # DERIVED: 124.0, inside owner's 115-125
CHASSIS_LENGTH = 101.0       # DERIVED: casing length minus drive band minus end clearance
CHASSIS_DISC_THICKNESS = 4.0 # ASSUMPTION: printed disc stiff enough at 124 mm
CHASSIS_FRAME_OD = 70.0      # DERIVED (R1): leaves a 29.5 mm radial pocket for a 28 mm motor

# Drive band: the chassis diameter is locally reduced here and carries the belt (R1).
DRIVE_BAND_OD = 90.0         # DERIVED: 9:1 with a 10 mm pulley, 7.4 mm wall clearance
DRIVE_BAND_WIDTH = 15.0      # DERIVED: axial budget

# --------------------------------------------------------------------- belt and pulley

BELT_WIDTH = 6.0             # ASSUMPTION: GT2 6 mm, the common size
BELT_BACK_THICKNESS = 1.4    # ASSUMPTION: GT2 nominal
BELT_TOOTH_HEIGHT = 0.75     # ASSUMPTION: GT2 nominal
DRIVE_PULLEY_OD = 10.0       # DERIVED: sets the 9:1 ratio

# ------------------------------------------------------------------------------ axis

# The casing rotates on bearings around a hollow boss on each chassis disc. The wheel
# motor sits on the centreline inside that boss with its shaft protruding outward.
#
# ASSUMPTION, and the weakest part of this model: how the wheel actually mounts and is
# driven is not resolved by any analysis so far. Treat the boss and bearing sizes as a
# placeholder that makes the parts printable, not as a design.
AXIS_BOSS_OD = 20.0          # ASSUMPTION: bearing bore rides on this
AXIS_BOSS_LENGTH = 12.0      # ASSUMPTION
AXIS_BOSS_BORE = 13.0        # ASSUMPTION: clears an N20 motor body (12 mm)
BEARING_OD = 27.0            # ASSUMPTION: 6704ZZ, 20 x 27 x 4
BEARING_ID = 20.0            # ASSUMPTION
BEARING_WIDTH = 4.0          # ASSUMPTION

# ----------------------------------------------------------------------------- wheels

# ASSUMPTION, and worth a decision: the wheel OD must exceed the casing OD or the casing
# drags. Ground clearance under the casing is (WHEEL_OD - CASING_OD) / 2.
WHEEL_OD = 150.0             # ASSUMPTION: gives 7.5 mm ground clearance
WHEEL_WIDTH = 18.0           # DERIVED: axial budget
WHEEL_BORE = 3.0             # ASSUMPTION: N20 output shaft
WHEEL_ORING_CROSS_SECTION = 3.0   # ASSUMPTION: O-ring tread (R7 trick)
WHEEL_HUB_OD = 20.0          # ASSUMPTION
WHEEL_SPOKE_COUNT = 5        # ASSUMPTION: cosmetic and mass reduction

# ------------------------------------------------------------------------------ camera

CAMERA_APERTURE_WIDTH = 18.0     # ASSUMPTION: clears the Camera Module 3 Wide lens barrel
CAMERA_APERTURE_HEIGHT = 14.0    # ASSUMPTION
CAMERA_MOUNT_HOLE_SPACING_X = 21.0   # ASSUMPTION: confirm against the module drawing
CAMERA_MOUNT_HOLE_SPACING_Y = 12.5   # ASSUMPTION
CAMERA_MOUNT_HOLE_DIA = 2.2      # ASSUMPTION: M2 clearance
CAMERA_MOUNT_PLATE_THICKNESS = 3.0

# --------------------------------------------------------------------------------- IMU

# R2: radial offset under 3 mm. The constraint is radial only, so the bridge reaches the
# centreline at an axial station clear of the hub.
IMU_PAD_SIZE = 16.0
IMU_BRIDGE_THICKNESS = 3.0
IMU_BRIDGE_WIDTH = 12.0
IMU_MOUNT_HOLE_DIA = 2.2         # ASSUMPTION: M2 clearance

# ------------------------------------------------------------------- fasteners, fixtures

M3_CLEARANCE = 3.4
M3_TAP = 2.5
M2_CLEARANCE = 2.2

TRIM_BOSS_COUNT = 6             # R4
TRIM_BOSS_RADIUS = 55.0         # R4
TRIM_BOSS_OD = 7.0
TRIM_BOSS_HEIGHT = 5.0

PENDULUM_DATUM_RADIUS = 50.0    # R8: known offset for the inertia measurement
PENDULUM_DATUM_DIA = 3.2

SLIP_RING_ENVELOPE_DIA = 15.0   # R5
SLIP_RING_ENVELOPE_LENGTH = 25.0

END_CAP_THICKNESS = 4.0
END_CAP_SCREW_COUNT = 6
END_CAP_SCREW_RADIUS = 61.0     # DERIVED: inside the 64.5 mm inner wall

STANDOFF_COUNT = 4              # ASSUMPTION: M3 standoffs joining the chassis discs
STANDOFF_RADIUS = 40.0          # DERIVED: must sit inside the drive band's 45 mm outer
                                # radius with clearance, and inside the chassis disc
                                # spokes. At 45 the M3 holes ran 1.7 mm off the band --
                                # caught by test_drive_band_diameter_gives_the_intended_ratio

MOTOR_BRACKET_THICKNESS = 4.0
PITCH_MOTOR_BORE = 28.0         # ASSUMPTION: 28 mm gimbal motor; the number R1 says to verify
PITCH_MOTOR_BOLT_RADIUS = 14.5  # ASSUMPTION
PITCH_MOTOR_BOLT_COUNT = 4

# --------------------------------------------------------------------------- materials

# For mass and inertia reporting only.
PLA_DENSITY = 1.24e-3           # g/mm^3
PETG_DENSITY = 1.27e-3
ABS_DENSITY = 1.04e-3
PRINT_DENSITY = ABS_DENSITY     # what the reports assume; ABS is 18% lighter than
                                # PETG, which is a free lever on rotating inertia
