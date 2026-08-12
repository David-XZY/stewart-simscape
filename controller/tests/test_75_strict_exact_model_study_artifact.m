function test_75_strict_exact_model_study_artifact
% Validate the reproducible exact nonlinear study and its guarantee scope.
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
resultFile = fullfile(projectRoot, 'results', 'reports', ...
    'strict_clf_cbf_qp', 'exact_model_study', ...
    'strict_clf_cbf_qp_exact_model_study.mat');
assert(isfile(resultFile), ...
    'Run run_14_strict_clf_cbf_qp_exact_model_study before this test.');
sample = load(resultFile, 'result');
result = sample.result;

assert(numel(result.runs) == 6);
assert(isequal(result.ablationIndices, [1, 2, 3]));
assert(isequal(result.robustnessIndices, [3, 4, 5, 6]));
assert(result.strictConfig.cbf.collisionAlpha1 == 12);
assert(result.strictConfig.cbf.collisionAlpha2 == 12);
guaranteeFlags = arrayfun(@(item) ...
    item.scenario.guaranteeApplicable, result.runs);
assert(isequal(find(guaranteeFlags), 3));

baseline = result.runs(1);
clfOnly = result.runs(2);
matched = result.runs(3);
assert(baseline.metrics.minActualCollisionMargin < 0);
assert(clfOnly.metrics.minActualCollisionMargin < 0);
assert(matched.metrics.strictMatchedPassed);
assert(matched.metrics.fullTrajectoryCompleted);
assert(matched.metrics.infeasibleCount == 0);
assert(matched.metrics.fallbackCount == 0);
assert(matched.metrics.commandCbfViolationCount == 0);
assert(matched.metrics.realizedCbfViolationCount == 0);
assert(matched.metrics.minActualCollisionMargin >= -2e-6);
assert(matched.metrics.minSigma >= result.strictConfig.sigmaSafe-2e-6);
assert(matched.metrics.maxLegAcceleration <= ...
    max(result.strictConfig.legAccelerationLimit)+2e-6);
assert(matched.metrics.qpTimeP95 <= 0.010);

for index = 1:numel(result.runs)
    run = result.runs(index);
    assert(run.metrics.maxSigmaLowerBoundExcess <= 5e-10);
    assert(run.metrics.maxCollisionCertificateExcess <= 5e-6);
    assert(all(isfinite(run.control.state), 'all'));
    assert(all(isfinite(run.dense.state), 'all'));
end

for index = 4:6
    assert(~result.runs(index).scenario.guaranteeApplicable);
end
end
