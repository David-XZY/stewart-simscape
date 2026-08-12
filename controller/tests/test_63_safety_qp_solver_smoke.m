function test_63_safety_qp_solver_smoke
% test_63_safety_qp_solver_smoke - 验证 SC-QP 求解器简单场景可运行
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'safety_qp_control'));

model = buildOptModelCustom();
config = makeSafetyQpControlConfig(model, struct('nominalMode', "pd"));
state = makeState(model, 0.002);
Fprev = zeros(6, 1);
Fnom = 0.25 * ones(6, 1);

[Fcmd, diagnostic] = stepSafetyQpController( ...
    state.q, state.qd, state.qRef, state.qdRef, state.qddRef, Fprev, model, config, Fnom);

assert(isequal(size(Fcmd), [6, 1]));
assert(all(isfinite(Fcmd)));
assert(all(Fcmd <= config.forceMax + 1e-9));
assert(all(Fcmd >= config.forceMin - 1e-9));
assert(isfield(diagnostic, 'status'));
assert(isfield(diagnostic, 'exitflag'));
assert(isfield(diagnostic, 'activeConstraints'));
assert(isfield(diagnostic, 'clfViolation'));
assert(isfield(diagnostic, 'cbfViolation'));
assert(isfield(diagnostic, 'solveTime'));
assert(isfield(diagnostic, 'safetyMargin'));
assert(isfinite(diagnostic.solveTime));
end

function state = makeState(model, offset)
state = struct();
state.q = model.qHome + offset * [1; -1; 0.5; 0.1; -0.1; 0.05];
state.qd = zeros(6, 1);
state.qRef = model.qHome;
state.qdRef = zeros(6, 1);
state.qddRef = zeros(6, 1);
end
