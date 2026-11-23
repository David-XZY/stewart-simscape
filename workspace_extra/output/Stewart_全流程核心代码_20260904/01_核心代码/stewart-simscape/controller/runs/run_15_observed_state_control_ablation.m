function result = run_15_observed_state_control_ablation(trajectoryFile, outputDir, options)
% run_15_observed_state_control_ablation - Four matched state-source cases.
arguments
    trajectoryFile {mustBeTextScalar} = ""
    outputDir {mustBeTextScalar} = ""
    options struct = struct()
end

runRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(runRoot);
projectRoot = fileparts(controllerRoot);
addpath(runRoot);
settings = struct('sampleTime', 0.01, 'stopTime', [], ...
    'baselineOverrides', struct(), 'strictOverrides', struct(), ...
    'plantPerturbation', struct(), 'failOnRejected', false);
settings = mergeKnownFields(settings, options);
if strlength(string(outputDir)) == 0
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(projectRoot, 'results', 'reports', ...
        ['observed_state_control_ablation_', timestamp]);
end
outputDir = char(outputDir);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

modes = ["legacy-direct", "observed-current-state"];
modeResults = repmat(struct('mode', "", 'run', struct()), 2, 1);
for modeIndex = 1:2
    runOptions = settings;
    runOptions.stateFeedbackMode = modes(modeIndex);
    runOptions.scenarios = ["ideal-force", "nonideal-force"];
    runOptions.failOnRejected = false;
    modeResults(modeIndex).mode = modes(modeIndex);
    modeResults(modeIndex).run = run_13_strict_clf_cbf_qp_simscape( ...
        trajectoryFile, outputDir, runOptions);
end

rows = repmat(struct(), 2, 1);
scenarios = ["ideal-force", "nonideal-force"];
for scenarioIndex = 1:2
    legacy = findReport(modeResults(1).run, scenarios(scenarioIndex));
    observed = findReport(modeResults(2).run, scenarios(scenarioIndex));
    rows(scenarioIndex).scenario = scenarios(scenarioIndex);
    rows(scenarioIndex).legacyTranslationRms = legacy.metrics.translationRms;
    rows(scenarioIndex).observedTranslationRms = observed.metrics.translationRms;
    rows(scenarioIndex).translationRmsChangePercent = percentChange( ...
        legacy.metrics.translationRms, observed.metrics.translationRms);
    rows(scenarioIndex).legacyTranslationPeak = legacy.metrics.translationPeak;
    rows(scenarioIndex).observedTranslationPeak = observed.metrics.translationPeak;
    rows(scenarioIndex).translationPeakChangePercent = percentChange( ...
        legacy.metrics.translationPeak, observed.metrics.translationPeak);
    rows(scenarioIndex).legacyRotationRms = legacy.metrics.rotationRms;
    rows(scenarioIndex).observedRotationRms = observed.metrics.rotationRms;
    rows(scenarioIndex).rotationRmsChangePercent = percentChange( ...
        legacy.metrics.rotationRms, observed.metrics.rotationRms);
    rows(scenarioIndex).legacyRotationPeak = legacy.metrics.rotationPeak;
    rows(scenarioIndex).observedRotationPeak = observed.metrics.rotationPeak;
    rows(scenarioIndex).rotationPeakChangePercent = percentChange( ...
        legacy.metrics.rotationPeak, observed.metrics.rotationPeak);
    rows(scenarioIndex).observedControllerStepP95 = ...
        observed.metrics.controllerStepTime95;
    rows(scenarioIndex).observedFallbackCount = observed.metrics.fallbackCount;
    rows(scenarioIndex).observedStrictGuaranteeApplicable = ...
        observed.strictGuaranteeApplicable;
    rows(scenarioIndex).legacyFullTrajectoryCompleted = ...
        legacy.fullTrajectoryCompleted;
    rows(scenarioIndex).observedFullTrajectoryCompleted = ...
        observed.fullTrajectoryCompleted;
    rows(scenarioIndex).observedPassed = observed.passed;
    rows(scenarioIndex).degradationWithinFivePercent = all([ ...
        rows(scenarioIndex).translationRmsChangePercent, ...
        rows(scenarioIndex).translationPeakChangePercent, ...
        rows(scenarioIndex).rotationRmsChangePercent, ...
        rows(scenarioIndex).rotationPeakChangePercent] <= 5);
end

summary = struct2table(rows);
writetable(summary, fullfile(outputDir, 'observed_state_ablation_summary.csv'));
result = struct('outputDir', string(outputDir), 'modeResults', modeResults, ...
    'summary', summary, 'allControlRunsPassed', ...
    all(arrayfun(@(item) item.run.allPassed, modeResults)), ...
    'fullTrajectoryCompleted', all(summary.legacyFullTrajectoryCompleted) && ...
    all(summary.observedFullTrajectoryCompleted), ...
    'degradationWithinFivePercent', all(summary.degradationWithinFivePercent));
result.passed = result.allControlRunsPassed && result.fullTrajectoryCompleted && ...
    result.degradationWithinFivePercent;
save(fullfile(outputDir, 'observed_state_ablation.mat'), ...
    'result', 'trajectoryFile', 'settings');
if settings.failOnRejected && ~result.passed
    error('run_15_observed_state_control_ablation:AcceptanceFailed', ...
        'At least one four-case observed-state ablation criterion failed.');
end
end

function report = findReport(runResult, scenario)
index = find([runResult.runs.scenario] == scenario, 1);
assert(~isempty(index), 'Missing ablation scenario %s.', scenario);
report = runResult.runs(index).report;
end

function value = percentChange(baseline, candidate)
value = 100*(candidate-baseline)/max(abs(baseline), eps);
end

function target = mergeKnownFields(target, source)
fields = fieldnames(source);
for index = 1:numel(fields)
    name = fields{index};
    if ~isfield(target, name)
        error('run_15_observed_state_control_ablation:UnknownOption', ...
            'Unknown run option: %s.', name);
    end
    target.(name) = source.(name);
end
end
