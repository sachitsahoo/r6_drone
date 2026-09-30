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
CHASSIS_FRAME_OD = 60.0      # DERIVED (R1): the motor's inner edge sits at radius 33.1, so
                             # the frame must stay under 62.3 mm OD to clear it

# Drive band: the chassis diameter is locally reduced here and carries the belt (R1).
# DERIVED: 8:1 with a 10 mm pulley. NOT 90 mm / 9:1 as first specified -- that checked
# only that the pulley cleared the shell, not the 28 mm motor body coaxial with it, which
# poked 1.7 mm through the wall. The largest band that fits a 28 mm motor with 2 mm margin
# is 83.7 mm OD. Caught by cad/assembly.py's clearance report.
DRIVE_BAND_OD = 80.0
DRIVE_BAND_WIDTH = 15.0      # DERIVED: axial budget

# --------------------------------------------------------------------- belt and pulley

BELT_WIDTH = 6.0             # ASSUMPTION: GT2 6 mm, the common size
BELT_BACK_THICKNESS = 1.4    # ASSUMPTION: GT2 nominal
BELT_TOOTH_HEIGHT = 0.75     # ASSUMPTION: GT2 nominal
DRIVE_PULLEY_OD = 10.0       # DERIVED: sets the 8:1 ratio

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
# 170, not 150. At 150 the ground clearance under the casing was 7.5 mm -- 10% of wheel
# radius -- which is very little for a robot that gets thrown and driven over rubble.
# Going to 170 costs 13.7 g on the pair (1.7% of the vehicle budget) and takes clearance to
# 17.5 mm. Wheel inertia rises 47%, but wheel inertia is only ~9% of the effective drive
# inertia, so the drive motors see about 15% more torque.
WHEEL_OD = 170.0             # DECIDED 2026-09-30; ground clearance 17.5 mm
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

# 6 mm, not 4: the bearing seat is 4 mm deep, so a 4 mm cap was bored straight through and
# left no shoulder for the bearing to seat against. Caught by cad/assembly.py.
END_CAP_THICKNESS = 6.0
#: The rim and spokes only carry screw loads, so they are thinner than the hub. The 6 mm
#: above is needed solely where the bearing seats: a 4 mm seat plus a 2 mm shoulder. Running
#: that thickness out to r=65 put ~7 g per cap at the largest radius in the rotating
#: assembly, which is the most expensive place in the machine to spend mass.
END_CAP_RIM_THICKNESS = 4.0

# REMOVED: a labyrinth lip that shrouded the chassis disc. It was added to stop the open
# wheels flinging grit into the 2.5 mm pitch gap, but that gap is already enclosed -- the
# cap's own rim closes the casing bore at each end, the shielded bearing closes the central
# bore, and the cap bolts to the shell so there is no relative motion at the rim to seal.
# The lip guarded a joint that does not open. Meanwhile the camera aperture is 252 mm^2 of
# hole straight through the shell, so the interior is not sealed by anything. If debris
# ingress turns out to matter, the aperture is where to solve it -- a window, not a lip.
#: Clearance bore through the cap's shoulder, so it passes the boss without rubbing.
END_CAP_BOSS_CLEARANCE_BORE = 21.0
END_CAP_SCREW_COUNT = 6
END_CAP_SCREW_RADIUS = 61.0     # DERIVED: inside the 64.5 mm inner wall

STANDOFF_COUNT = 4              # ASSUMPTION: M3 standoffs joining the chassis discs
# Computed, not chosen. This was a literal twice, and twice it collided with the drive
# band's outer radius after the band diameter changed -- the M3 holes ran off the edge of
# the part both times. Deriving it from the band and frame kills the whole class of bug:
# midway between the frame it bolts to and the band's rim.
STANDOFF_RADIUS = (CHASSIS_FRAME_OD / 2 + DRIVE_BAND_OD / 2) / 2

MOTOR_BRACKET_THICKNESS = 4.0
# 28 mm matches a **2208** gimbal motor (vendor listings: 28 mm OD, 39-42 g, 3 mm shaft).
# It does NOT match the more commonly recommended iPower GM2804 / GBM2804H, whose "2804"
# names the 28 x 04 mm STATOR -- its actual outer diameter is 35 mm, confirmed across four
# vendor listings. GM3506 is 40 mm. Either forces the drive band and chassis frame smaller:
#
#   motor OD 28 -> band 80, frame 60, 8.0:1   (the current design)
#   motor OD 35 -> band 75, frame 50, 7.5:1
#   motor OD 40 -> band 70, frame 40, 7.0:1
#
# So this value was right by luck, not by design. See ADR 0006's candidate list.
PITCH_MOTOR_BORE = 28.0
#: Axial length of the motor body. Confirmed from the SpeedyFPV 2208 listing: 26 mm.
#: This was never modelled or checked -- only the diameter was. It fits with 46 mm to
#: spare in the annular pocket, but nothing was verifying that until the real part
#: was looked up.
PITCH_MOTOR_LENGTH = 26.0
PITCH_MOTOR_MASS_G = 39.0
PITCH_MOTOR_KV = 80.0
PITCH_MOTOR_BOLT_RADIUS = 14.5  # ASSUMPTION
PITCH_MOTOR_BOLT_COUNT = 4

# --------------------------------------------------------------------------- materials

# For mass and inertia reporting only.
PLA_DENSITY = 1.24e-3           # g/mm^3
PETG_DENSITY = 1.27e-3
ABS_DENSITY = 1.04e-3
PRINT_DENSITY = ABS_DENSITY     # what the reports assume; ABS is 18% lighter than
                                # PETG, which is a free lever on rotating inertia


# ------------------------------------------------- derived drive geometry (do not edit)

BELT_TOTAL_THICKNESS = BELT_BACK_THICKNESS + BELT_TOOTH_HEIGHT

#: Radius at which the drive pulley's axis sits, riding on the belt bonded to the band.
DRIVE_PULLEY_CENTER_RADIUS = (DRIVE_BAND_OD / 2) + BELT_TOTAL_THICKNESS + (DRIVE_PULLEY_OD / 2)

#: Reduction from motor to casing.
DRIVE_RATIO = DRIVE_BAND_OD / DRIVE_PULLEY_OD

#: Radial extent of the pitch motor body, coaxial with the pulley. Both must clear the
#: casing wall outboard and the chassis frame inboard.
PITCH_MOTOR_OUTER_RADIUS = DRIVE_PULLEY_CENTER_RADIUS + PITCH_MOTOR_BORE / 2
PITCH_MOTOR_INNER_RADIUS = DRIVE_PULLEY_CENTER_RADIUS - PITCH_MOTOR_BORE / 2

#: Motor bracket, sized to span the annular pocket without touching either wall.
MOTOR_BRACKET_RADIAL_SPAN = 32.0
MOTOR_BRACKET_TANGENTIAL_SPAN = 40.0
