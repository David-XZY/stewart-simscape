function test_78_lqi_schedule_runtime_contract
% test_78_lqi_schedule_runtime_contract - Saturation and anti-windup update.
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
addpath(fullfile(controllerRoot, 'lqi_schedule_comparison'));

sample = struct('controllerA', [0.8, 0; 0, 0.7], ...
    'controllerB', [1, 0; 0, 1], 'controllerC', [3, -2; 1, 4], ...
    'controllerD', zeros(2, 12));
sample.controllerA = blkdiag(sample.controllerA, zeros(16));
sample.controllerB = [sample.controllerB, zeros(2, 10); zeros(16, 12)];
sample.controllerC = [sample.controllerC, zeros(2, 16); zeros(4, 18)];
sample.controllerD = zeros(6, 12);
state = [2; -1; zeros(16, 1)];
error = [0.1; -0.2; zeros(4, 1)];
oldAntiWindup = zeros(6, 1);
[nextState, nextAntiWindup, raw, saturated] = stepDiscreteLqiController( ...
    sample, state, error, oldAntiWindup, 1);
assert(isequal(raw, [8; -2; zeros(4, 1)]));
assert(isequal(saturated, [1; -1; zeros(4, 1)]));
assert(isequal(nextAntiWindup, [-7; 1; zeros(4, 1)]));
expectedState = sample.controllerA*state+sample.controllerB*[error; nextAntiWindup];
assert(max(abs(nextState-expectedState)) < eps);
end
