function test_88_qp_aware_lqi_antiwindup_contract
% Final QP output must replace local-only LQI anti-windup mismatch.
controllerRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(controllerRoot, 'lqi_schedule_comparison'));
sample = struct();
sample.controllerA = 0.9*eye(18);
sample.controllerB = [eye(12); zeros(6, 12)];
state = (1:18).'/100;
poseError = [0.1; -0.2; 0.3; -0.1; 0.2; -0.3];
rawFeedback = [80; -70; 60; -50; 40; -30];
feedforward = 300*ones(6, 1);
dobForce = [20; -15; 10; -5; 2; -1];
qpDelta = [-35; 25; -15; 10; -8; 6];
applied = feedforward+dobForce+rawFeedback+qpDelta;
[nextState, mismatch, diagnostic] = closeLqiAntiWindupWithAppliedForce( ...
    sample, state, poseError, rawFeedback, applied, feedforward, dobForce, 30);
expected = min(max(qpDelta, -30), 30);
assert(max(abs(mismatch-expected)) < 1e-12);
assert(max(abs(nextState-(sample.controllerA*state+ ...
    sample.controllerB*[poseError; expected]))) < 1e-12);
assert(max(abs(diagnostic.equivalentAppliedFeedback- ...
    (rawFeedback+qpDelta))) < 1e-12);
assert(diagnostic.limitActive);

[~, scaledMismatch] = closeLqiAntiWindupWithAppliedForce( ...
    sample, state, poseError, rawFeedback, applied, feedforward, dobForce, ...
    30, 0.1);
assert(max(abs(scaledMismatch-0.1*qpDelta)) < 1e-12);

[identityState, identityMismatch] = closeLqiAntiWindupWithAppliedForce( ...
    sample, state, poseError, rawFeedback, ...
    feedforward+dobForce+rawFeedback, feedforward, dobForce, 30);
assert(max(abs(identityMismatch)) < 1e-12);
assert(max(abs(identityState-(sample.controllerA*state+ ...
    sample.controllerB*[poseError; zeros(6, 1)]))) < 1e-12);
fprintf('test_88_qp_aware_lqi_antiwindup_contract passed.\n');
end
