function test_82_observed_state_simscape_smoke
% Candidate mode must complete without truth fallback in a short run.
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
addpath(fullfile(controllerRoot, 'runs'));
outputDir = tempname;
mkdir(outputDir);
result = run_13_strict_clf_cbf_qp_simscape('', outputDir, struct( ...
    'scenarios', 'ideal-force', ...
    'stateFeedbackMode', 'observed-current-state', 'stopTime', 0.1));
report = result.runs(1).report;
assert(report.stateFeedbackMode == "observed-current-state");
assert(report.strictGuaranteeApplicable == false);
assert(report.metrics.fallbackCount == 0);
assert(report.metrics.infeasibleCount == 0);
assert(report.metrics.solverErrorCount == 0);
assert(report.acceptance.observerFinitePassed);
assert(report.acceptance.allStepsFeasible);
assert(report.acceptance.noFallback);
assert(report.acceptance.noSolverError);
assert(report.acceptance.lengthPassed);
assert(report.acceptance.collisionPassed);
assert(report.observerMetrics.tailTranslationRms <= 0.05e-3);
assert(report.observerMetrics.tailRotationRms <= deg2rad(0.01));
assert(exist(fullfile(outputDir, ...
    'strict_qp_observed_current_state_ideal_force_tracking.csv'), 'file') == 2);
assert(exist(fullfile(outputDir, ...
    'strict_qp_observed_current_state_ideal_force_observer.csv'), 'file') == 2);
end
