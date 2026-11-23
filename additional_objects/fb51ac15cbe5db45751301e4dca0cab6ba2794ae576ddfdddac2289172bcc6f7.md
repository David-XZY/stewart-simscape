# LQI final-output anti-windup and command-governor ablation

Generated: 2026-08-12

## Implemented variants

- QP-aware LQI anti-windup computes the equivalent delivered feedback as
  `Fcmd - Ffeedforward - FDOB` and feeds its mismatch from the unsaturated LQI
  output into the existing anti-windup channel.
- The selected attenuated setting applies 10% of that mismatch and limits it
  to 30 N per leg. Both features remain disabled unless explicitly selected.
- The attitude command governor filters only the injected pose-command noise.
  Nominal motion and the smooth target bump remain unchanged. It is a
  critically damped 2 Hz second-order governor with per-axis rate limits
  `[2,2,3] deg/s` and acceleration limits `[20,20,30] deg/s^2`.
- Governed attitude, angular rate, and angular acceleration are derivative
  consistent, and computed-torque feedforward is recomputed from them.

## Direct anti-windup screening

Full-strength QP mismatch feedback was rejected. Across the four-case
representative Simscape matrix it reduced weighted tracking error by 1.87%,
but increased fallbacks from 2 to 7, introduced acceleration violations in
three cases, increased collision-violation cases from two to four, and raised
maximum P95 from 10.95 ms to 12.28 ms.

Reducing the cross-layer gain to 0.1 removed the extra fallback in the medium
noise case, but anti-windup alone still introduced five sampled collision
violations. It is therefore not recommended as a standalone controller
change.

## Selected 0.1-gain plus governor validation

The selected comparison covers medium target noise, high target noise, and
high wrench plus target noise under the same trajectory, model, actuator,
sample time, DOB parameters, and random seeds.

|Variant|Weighted error ratio|Fallbacks|Acceleration cases|Collision cases|Maximum P95|
|---|---:|---:|---:|---:|---:|
|DOB-aware strict-QP baseline|1.0000|2|0|2|11.95 ms|
|2 Hz command governor|1.2647|2|0|0|15.32 ms|
|Governor + 0.1-gain anti-windup|1.2645|1|0|0|12.02 ms|

The combined variant removes the medium-noise fallback and eliminates all
sampled collision violations in these three cases. However, relative to the
unfiltered target it increases weighted tracking error by 26.45% and still
misses the 10 ms online timing gate. On the two high-intensity cases, adding
anti-windup does not reduce fallback relative to the governor alone.

## Conclusion

Neither feature should replace the current controller yet:

- Standalone final-output anti-windup is rejected because it makes the LQI
  oppose safety-filter actions and creates new safety violations.
- The command governor is useful as an optional safety-oriented mode: it
  removes the observed collision-margin violations and sharply reduces force
  rate/leg acceleration, but its 2 Hz setting tracks the raw noisy target too
  conservatively.
- Their combination has a real but narrow benefit in the medium-noise case;
  it is not robustly better across high-intensity cases and remains too slow.

The next experiment should replace fixed low-pass tracking with a constraint-
aware governor that passes low-frequency target motion unchanged and only
attenuates the component predicted to violate leg acceleration or collision
CBFs. Cross-layer anti-windup should then use the sustained/low-frequency QP
correction rather than each instantaneous safety-filter action.

This is a representative ablation, not a new 40-case acceptance result.
