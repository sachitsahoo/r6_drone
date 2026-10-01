# Learning note — the wheel velocity loop

A plain-language walkthrough of [ADR 0013](../decisions/0013-wheel-velocity-loop.md) and
[`firmware/core/control/`](../../firmware/core/control/). The maths is in
[`docs/theory/wheel-velocity-loop.md`](../theory/wheel-velocity-loop.md).

## The job

The operator sends "go this fast, turn this fast" about 50 times a second. A thousand times
a second, the MCU has to turn that into a PWM duty for each wheel motor, so that each wheel
actually spins at the speed it should. The wheel is never told its speed directly; all the
loop has is an encoder counter that ticks as the wheel turns.

## The pipeline, one step at a time (every 1 ms)

1. **Kinematics** (`diff_drive`). Split the body command into a left and a right wheel speed.
   To turn left, the right wheel runs faster. If either wheel would exceed the speed cap, slow
   *both* by the same factor, so the robot drives the same arc, just slower.
2. **Acceleration limit** (`RateLimiter`). Don't jump to the new speed; ramp toward it at
   1 m/s². A sudden lurch swings the casing like a pendulum, which is the very thing the
   project is trying to keep the camera free of.
3. **Measure speed** (`WheelSpeedEstimator`). Count how many encoder ticks happened over the
   last 10 ms and divide by the time. Ten milliseconds, not one: over 1 ms the wheel only
   moves a couple of ticks, so the estimate would jump around in huge steps.
4. **Decide the duty** (`WheelVelocityController`). Compare the target with the measurement
   and push the motor: harder the further off it is (P), plus whatever extra a steady drag
   like carpet or a slope needs (I).
5. **Write the motor**, but only when ARMED. Otherwise reset and stay quiet. Stopping the motors
   is the safety state machine's job, not this loop's.

## The ideas worth being able to explain

- **The integrator learns the drag.** P alone can only push while it's still behind, so it
  always settles a bit slow under load. I keeps adding duty as long as there's any error,
  so it only stops when the error is exactly zero. On carpet it simply settles at a higher
  value. That's why one set of gains works on every floor.
- **Anti-windup.** If the motor is already at its duty limit, adding to I achieves nothing;
  it just stores up duty that causes an overshoot later. So I freezes while saturated, unless
  the error is pulling it back.
- **Why the gains have the values they do.** `ki/kp` is chosen so the controller's zero cancels
  the wheel's own lag. What's left behaves like the simplest possible loop, a first-order
  response with a 20 ms time constant. The window's 5 ms delay is what stops it going faster.
- **The mistake we caught.** Feedforward ("this speed usually needs this duty") sounded like
  pure upside. But the PI was already tuned to deliver the ideal response, so adding
  feedforward pushed the wheel twice and it overshot by 12%. The SIL test found it; the maths
  confirmed it; the default is now `kff = 0`.

## Check your understanding

1. **Why does the loop measure speed over 10 ms instead of the 1 ms between samples?**
   One encoder count over 1 ms is 4.49 rad/s, or 0.24 m/s at the rim: far too coarse. Over 10
   ms one count is 0.449 rad/s. The cost is about 5 ms of delay, which limits how fast the
   loop can safely be.

2. **Why is there no D term?** D differentiates the measured speed, and the measured speed
   moves in steps of 0.449 rad/s. One step between 1 ms samples looks like 449 rad/s² of
   acceleration, about 24 times the largest real acceleration (19 rad/s²). D would amplify
   noise, and the wheel, a first-order lag, doesn't need D's phase lead anyway.

3. **Carpet needs more duty than tile. Why don't we need different gains for each?**
   Carpet mostly adds a near-constant drag. The integrator rejects any constant load with
   zero steady error: it keeps growing until the error is gone. The gains set how fast the
   loop responds, and that depends mostly on the motor and gearbox, not the floor.

4. **Why was feedforward set to zero, if it predicts the needed duty?** With the PI zero
   cancelling the wheel's pole, the PI alone gives a clean first-order response. Feedforward
   adds a second path from the command to the motor. That puts a slow zero in the response,
   which caused 12% overshoot. Feedforward would only cut ramp lag from 20 to 11 mm/s.

5. **When does the integrator reset?** Only when the robot leaves ARMED. Disarming, a fault or
   an e-stop all reset it, so a stale value can't drive the next arming. Saturation *freezes*
   it. Zero commands, reversals and surface changes keep it.

6. **Why does the robot sag briefly when armed on a slope?** The integrator starts from zero
   on arming and needs a few tens of milliseconds to build up the holding duty. Preloading a
   remembered value was rejected: a stale integrator is the riskier failure.

7. **Why does saturation scale both wheels rather than clip the fast one?** The ratio of the
   wheel speeds sets the turning radius. Clipping one wheel changes the ratio, so the robot
   would follow a different arc than commanded. Scaling both keeps the arc and only slows it.

8. **Why does the integrator store duty (`I += ki·e·dt`) rather than the integral of error?**
   Gains can change while ARMED. If the output were `ki × ∫e`, changing ki would instantly
   rescale the whole history and jump the duty. Stored as duty, a new ki only changes future
   accumulation: bumpless.

9. **What happens on an encoder glitch?** A count jump no real wheel could make (faster than
   100 rad/s) is flagged as a fault, and the window restarts from the new reading. For that
   one step the controller outputs feedforward only, which is zero by default, with the
   integrator frozen, rather than act on a bad number. Deciding whether to disarm is the
   safety state machine's call.

10. **Why do we reject a slower disturbance recovery for the faster reference tracking?**
    Pole-zero cancellation makes command-following first order and clean, but a sudden load
    still recovers at the wheel's own 50 ms lag. For driving that's fine. If the stabilizer
    later needs a stiffer drive, raising ki/kp past cancellation buys faster recovery at the
    cost of some overshoot.
