# DOB-aware acceleration CBF validation

Generated: 2026-08-12

## Implemented model

The strict QP now optimizes a regulated leg-force component `Freg` while the
plant receives the total force

`Ftotal = Freg + FDOB`.

The acceleration-level CLF/CBF prediction uses

`qdd = f(q,qd) + B(q)*(Freg + FDOB) + H(q)^(-1)*What_d`.

Therefore the known drift contribution is

`a_known = B*FDOB + H^(-1)*What_d`.

Leg-acceleration, length HOCBF, speed CBF, collision HOCBF, singularity HOCBF,
and CLF constraints all use this same known contribution. Force magnitude and
force-rate bounds remain constraints on `Ftotal`, not on `Freg` alone.

When `Jv'*FDOB = -What_d`, the two DOB terms cancel in the prediction. If
force or rate limiting prevents full cancellation, the CBF sees only the
remaining estimated disturbance acceleration.

## Feasible-set contract test

A reproducible test uses 50 N per leg of DOB compensation, its exactly mapped
opposing generalized-wrench estimate, and a tight 10 N/s total-force slew
limit. The DOB-blind prediction cannot move far enough to remove the apparent
acceleration and falls back with minimum CBF residual `-0.519719`. The
DOB-aware formulation solves without fallback, preserves the previous total
force, and has minimum CBF residual `+0.454655`.

Zero DOB force and zero disturbance estimate reproduce the legacy strict-QP
force command and CBF bounds to numerical precision.

## Representative Simscape comparison

The following four cases use the same trajectory, model, actuator boundary,
sample time, random seed, 8 Hz DOB, 180 N leg compensation limit, and
2000 N/s DOB slew limit as the existing evidence. “Before” is the committed
pre-change 40-case result; “after” is a focused post-change rerun.

|Case|RMS before|RMS after|Acceleration violations|Collision violations|Fallbacks|After P95|
|---|---:|---:|---:|---:|---:|---:|
|nominal|0.000763405|0.000763478|0 -> 0|5 -> 0|0 -> 0|9.876 ms|
|target noise x1.0 seed 101|0.000927324|0.000922244|2 -> 0|0 -> 0|1 -> 1|11.957 ms|
|wrench + smooth target x0.5|0.000774514|0.000774188|0 -> 0|3 -> 0|1 -> 0|11.496 ms|
|wrench + target noise x1.5 seed 102|0.0182407|0.000990570|5 -> 0|11 -> 3|2 -> 0|13.276 ms|

## Conclusion and evidence boundary

The change fixes the intended feasibility-model defect: the synthetic
feasible-set reproduction changes from failed/fallback to solved, and three
of four representative Simscape cases eliminate all previous QP fallbacks.
The high-intensity combined case reduces equivalent-pose RMS by 94.57% while
removing both fallbacks and all five acceleration violations.

It is not yet a full controller acceptance result. The medium target-noise
case retains one fallback, the high combined case retains three sampled
collision-margin violations, and post-change P95 exceeds 10 ms in three of
the four representative runs. The original 40-case ranking must not be
relabeled as post-change evidence; a full matched 40-case rerun is still
required before making a new overall superiority claim.
