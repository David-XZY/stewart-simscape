function test_70_strict_clf_cbf_qp_config_contract
% Contract: controller type 11, one CLF slack, and physical hard limits.
[model, ~, controllerRoot] = testEnvironment();
addpath(fullfile(controllerRoot, 'strict_clf_cbf_qp'));

config = makeStrictClfCbfQpConfig(model, struct( ...
    'dt', 0.005, 'time', 1.25, ...
    'cbf', struct('lengthAlpha1', 3.5)));

assert(config.controllerType == 11);
assert(config.shortName == "S-CLF-CBF-QP");
assert(config.dt == 0.005);
assert(config.time == 1.25);
assert(config.cbf.lengthAlpha1 == 3.5);
assert(config.cbf.collisionAlpha1 == 12.0);
assert(config.cbf.collisionAlpha2 == 12.0);
assert(isequal(size(config.clf.P), [12, 12]));
assert(min(eig(0.5*(config.clf.P+config.clf.P.'))) > 0);
assert(numel(config.forceMin) == 6 && all(config.forceMin < config.forceMax));
assert(all(config.forceRateLimit > 0));
assert(config.fallback.isSafetyCertified == false);
assert(exist('stepStrictClfCbfQp', 'file') == 2);
assert(exist('buildStrictClfCbfAdModel', 'file') == 2);

newPositionGain = diag([24, 25, 26, 14, 15, 16]);
gainOverride = makeStrictClfCbfQpConfig(model, struct( ...
    'clf', struct('positionGain', newPositionGain)));
assert(isequal(gainOverride.clf.positionGain, newPositionGain));
assert(norm(gainOverride.clf.P-config.clf.P, 'fro') > 1e-6);

explicitP = 2*eye(12);
explicitOverride = makeStrictClfCbfQpConfig(model, struct( ...
    'clf', struct('positionGain', newPositionGain, 'P', explicitP)));
assert(isequal(explicitOverride.clf.P, explicitP));

assertThrows(@() makeStrictClfCbfQpConfig(model, struct( ...
    'cbf', struct('collisionAlpha1', 0))), ...
    'MATLAB:makeStrictClfCbfQpConfig:expectedPositive');
end

function assertThrows(operation, expectedIdentifier)
didThrow = false;
try
    operation();
catch exception
    didThrow = true;
    assert(strcmp(exception.identifier, expectedIdentifier), ...
        'Unexpected error identifier: %s', exception.identifier);
end
assert(didThrow, 'Expected operation to throw an exception.');
end

function [model, projectRoot, controllerRoot] = testEnvironment()
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
model = buildOptModelCustom();
end
