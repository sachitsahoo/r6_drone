# Lessons

[2026-09-30] | Proposed a custom ring PCB for the IMU; CLAUDE.md puts custom PCBs out of scope | Check the out-of-scope list before proposing any custom hardware
[2026-09-30] | Quoted bearing part numbers and motor specs from memory; 61807 was wrong | Look up every bought-part number against a listing before writing it into CAD or an ADR, and cite the source
[2026-09-30] | Bolted the motor stator (windings) to the chassis, sending phase leads across the joint | For any part spanning a rotating joint, ask which side its wires come out of before choosing its orientation
[2026-09-30] | Told the owner the IMU offset only mattered in tumbles; ignored the tangential alpha*r term | When budgeting an error, enumerate every term in the formula before calling one negligible
[2026-09-30] | Interference sweep checked one pose of parts that rotate relative to each other | Any clearance check between bodies that move relative to each other must cover the full range of motion
[2026-09-30] | A copied figure (torque budget) went stale silently when the design changed | When a number must be copied across a boundary, add a test that fails if the copy drifts
[2026-10-01] | ADR 0013 claimed PI pole-zero cancellation gives a first-order closed loop, but the design also fed the reference forward; that adds a closed-loop zero and 12% step overshoot, caught only by the SIL test | Derive the reference-to-output transfer function through every path the reference takes (feedforward, setpoint weighting) before stating a response shape or setting a test bound
[2026-10-01] | ADR 0014 (written and accepted earlier) said 500 ms is "about 10" time constants of 20 ms; it is 25. Caught only while writing the theory doc | Recompute every ratio and product quoted in an ADR before submitting it; never write "about N" without doing the division
[2026-10-01] | After a mutation test, restoring the source with cp left the build "up to date" and a stale mutant binary ran; one round's result was misread before rerunning | After restoring files, touch them and confirm the rebuild actually recompiled before trusting a test result
