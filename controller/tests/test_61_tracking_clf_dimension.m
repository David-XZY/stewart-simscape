function test_61_tracking_clf_dimension
% test_61_tracking_clf_dimension - 验证 CLF 约束输出维度与诊断量
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'safety_qp_control'));

model = buildOptModelCustom();
config = makeSafetyQpControlConfig(model, struct('nominalMode', "pd"));
q = model.qHome + [0.002; -0.001; 0.0015; 0.001; -0.0005; 0.0008];
qd = [0.01; -0.02; 0.015; 0.001; -0.0015; 0.002];
qRef = model.qHome;
qdRef = zeros(6, 1);
qddRef = zeros(6, 1);

clf = buildTrackingClf(q, qd, qRef, qdRef, qddRef, model, config);

assert(isequal(size(clf.A), [1, 6]));
assert(isscalar(clf.b));
assert(isscalar(clf.V));
assert(isscalar(clf.VdotNominal));
assert(isfinite(clf.V));
assert(clf.V > 0);
assert(isfinite(clf.b));
assert(isfield(clf, 'desiredAcceleration'));
assert(isequal(size(clf.desiredAcceleration), [6, 1]));
end
