function limit=ch5PredictionAccelerationLimit(r)
% Conservative prediction band; the actual physical limit is unchanged.
% A development probe reached the nominal 1.2 m/s^2 boundary, while the
% measured Simscape acceleration exceeded it by 2.01e-4 m/s^2. Reserve one
% percent for model/integration/discretization error. This engineering
% margin is not a certified disturbance bound.
limit=.99*r.physical.actuator.lddotMax;
end
