function test_64_safety_qp_fallback
% test_64_safety_qp_fallback - 验证 QP 不可行时 fallback 不中断且力限幅有效
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'safety_qp_control'));

model = buildOptModelCustom();
config = makeSafetyQpControlConfig(model, struct( ...
    'forceMin', -ones(6, 1), ...
    'forceMax', ones(6, 1), ...
    'forceRateLimit', 1e-6, ...
    'dt', 0.001, ...
    'forceInfeasibleForTest', true));
q = model.qHome + [0.01; 0; 0; 0; 0; 0];
qd = zeros(6, 1);
qRef = model.qHome;
qdRef = zeros(6, 1);
qddRef = zeros(6, 1);
Fprev = zeros(6, 1);
Fnom = 100 * ones(6, 1);

[Fcmd, diagnostic] = stepSafetyQpController( ...
    q, qd, qRef, qdRef, qddRef, Fprev, model, config, Fnom);

assert(isequal(size(Fcmd), [6, 1]));
assert(all(isfinite(Fcmd)));
assert(all(Fcmd <= config.forceMax + 1e-9));
assert(all(Fcmd >= config.forceMin - 1e-9));
assert(diagnostic.usedFallback);
assert(diagnostic.safetyBrakeCount == 1);
assert(diagnostic.status == "fallback");
end
