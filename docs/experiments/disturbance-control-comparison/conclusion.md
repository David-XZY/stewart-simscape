# Stewart platform disturbance-control comparison

Generated: 2026-08-12 14:16:52

Equivalent error: `sqrt(||e_p||^2 + (0.5 ||e_r||)^2)`.

All ranked controllers used the same trajectory, initial state, model, actuator boundary, 0.01 s sample time, case list, and random seeds.

## Ranking

|Rank|Controller|Total ratio|Improvement|Eligible|Better than existing|
|---:|---|---:|---:|:---:|:---:|
|1|scheduled_lqi_dob|0.300045|70.00%|0|0|
|2|scheduled_lqi_dob_strict_qp|0.368367|63.16%|0|0|
|3|fixed_lqi|1|0.00%|0|0|
|4|strict_qp|70.9187|-6991.87%|0|0|

## Conclusion

No controller met every 'better than existing control' gate in Simscape. The ranked errors remain useful, but no superiority claim is made.

## Acceptance diagnostics

|Controller|Completed|Hard-pass cases|Acceleration violation cases|Force-rate violation cases|Collision violation cases|Fallbacks|Infeasible|Worst P95 (ms)|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|fixed_lqi|40/40|0/40|13|15|40|0|0|0|
|scheduled_lqi_dob|40/40|0/40|22|8|40|0|0|2.157|
|strict_qp|40/40|25/40|15|0|12|3649|3649|8.823|
|scheduled_lqi_dob_strict_qp|40/40|9/40|14|0|24|29|29|10.79|

## Engineering conclusion and next optimization boundary

- `scheduled_lqi_dob` is the tracking winner, but it is not an accepted controller: high-intensity target noise causes leg-acceleration violations, some cases exceed the force-rate boundary, and the physical collision margin remains negative.
- `scheduled_lqi_dob_strict_qp` removes the force-rate issue and greatly reduces total tracking error, but it still has acceleration/collision violations, QP infeasible/fallback events, and a worst-case P95 above 10 ms.
- The current `strict_qp` is not robust to pose-command noise in this matrix; fallback-heavy cases dominate its aggregate error. It should not be described as a tracking improvement over fixed LQI.
- Next optimization should be constrained, not score-only: add a target-command prefilter/command governor with feedforward-consistent derivatives, explicitly include DOB compensation in acceleration CBF feasibility, and reduce QP execution cost before retuning tracking weights.

## Appendix boundary

Ideal leg-length PIDF and pose-to-leg-length cascade use a different actuator/control boundary. They remain appendix references and are not included in the total ranking.

## Evidence boundary

A configuration is called better only if it reduces the aggregate weighted error by at least 15%, has no medium/high combined-case regression above 10%, passes all hard constraints without fallback/nonfinite values, and keeps online P95 at or below 10 ms.
