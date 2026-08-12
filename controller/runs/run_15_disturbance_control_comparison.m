function result = run_15_disturbance_control_comparison(trajectoryFile, outputDir, options)
% run_15_disturbance_control_comparison - Unified disturbance/control study.
%
% The source SLX is never saved.  References, platform disturbances, and
% controller variants are assembled at runtime.  Full MAT evidence is kept
% under ignored results/, while CSV/Markdown/PNG evidence is exported to docs/.
arguments
    trajectoryFile {mustBeTextScalar} = ""
    outputDir {mustBeTextScalar} = ""
    options struct = struct()
end

runRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(runRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'strict_clf_cbf_qp'));
addpath(fullfile(controllerRoot, 'exact_model_study'));
addpath(fullfile(controllerRoot, 'lqi_schedule_comparison'));
addpath(fullfile(controllerRoot, 'disturbance_control_comparison'));
ensureLocalCasadiRuntime(projectRoot);

config = makeDisturbanceComparisonConfig(options);
if strlength(string(trajectoryFile)) > 0
    config.trajectoryFile = char(trajectoryFile);
end
if strlength(string(outputDir)) > 0
    config.outputDir = char(outputDir);
end

baselineOverrides = config.baselineOverrides;
baselineOverrides.controlLaw = 'computed-torque';
baselineOverrides.actuatorMode = 'ideal-force';
baselineOverrides.derivativeSampleTime = config.sampleTime;
setup = prepareSimscapePoseForceControl(config.trajectoryFile, baselineOverrides);
cleanup = onCleanup(@() closeWithoutSaving(setup.modelName));
nominalReference = makeReference(setup.refs);
if abs(median(diff(nominalReference.time))-config.sampleTime) > 1e-11
    error('run_15_disturbance_control_comparison:SampleTimeMismatch', ...
        'Prepared reference sample time does not match config.sampleTime.');
end

fixedSchedule = buildReferenceScheduledLqi(nominalReference, setup.model, ...
    setup.config, 'mode', 'fixed', 'stateStep', config.stateStep, ...
    'inputStep', config.inputStep);
scheduledSchedule = buildReferenceScheduledLqi(nominalReference, setup.model, ...
    setup.config, 'mode', 'scheduled', 'stateStep', config.stateStep, ...
    'inputStep', config.inputStep);
[strictFilterScene, strictConfig] = makeDisturbanceStrictFilter( ...
    setup.model, setup.scene, config);
warmStrictQp(nominalReference, setup, strictFilterScene, strictConfig);

allCases = buildDisturbanceCaseMatrix(config, 'all');
if ~isempty(config.caseIds)
    allCases = allCases(ismember([allCases.id], string(config.caseIds)));
    if isempty(allCases)
        error('run_15_disturbance_control_comparison:EmptyCaseSelection', ...
            'caseIds selected no experiment cases.');
    end
end
trainingCases = buildDisturbanceCaseMatrix(config, 'training');
trainingCases = trainingCases(ismember([trainingCases.id], [allCases.id]));
scheduleCache = struct('nominal', scheduledSchedule);
defaultDob = makeDobConfig(config.sampleTime, 5, 120, 2000);

fprintf('\n===== Stage 1: existing controllers, full exact matrix =====\n');
exactConfig = config;
exactConfig.screeningSkipCollision = false;
baselineControllers = [makeController("fixed_lqi", fixedSchedule, false, false, ...
        defaultDob, setup.scene); ...
    makeController("strict_qp", fixedSchedule, false, true, defaultDob, ...
        strictFilterScene)];
[baselineRuns, scheduleCache] = runCases(baselineControllers, allCases, ...
    nominalReference, scheduleCache, setup, strictConfig, exactConfig);

fprintf('\n===== Stage 2a: DOB grid search on training cases =====\n');
[bestDob, tuning] = tuneDob(trainingCases, baselineRuns, setup, config);

fprintf('\n===== Stage 2b: candidate screening on exact nonlinear model =====\n');
screenConfig = config;
screenConfig.screeningSkipCollision = true;
candidateControllers = [ ...
    makeController("scheduled_lqi", scheduledSchedule, false, false, bestDob, setup.scene); ...
    makeController("fixed_lqi_dob", fixedSchedule, true, false, bestDob, setup.scene); ...
    makeController("scheduled_lqi_dob", scheduledSchedule, true, false, bestDob, setup.scene); ...
    makeController("scheduled_lqi_dob_strict_qp", scheduledSchedule, true, true, ...
        bestDob, strictFilterScene)];
[candidateScreenRuns, scheduleCache] = runCases(candidateControllers, ...
    trainingCases, nominalReference, scheduleCache, setup, strictConfig, screenConfig);
baselineTraining = selectCases(baselineRuns, [trainingCases.id]);
screenTable = summarizeDisturbanceRuns([baselineTraining; candidateScreenRuns], ...
    "exact_screening");
screenRanking = rankDisturbanceControllers(screenTable, config);
candidateRows = screenRanking(ismember(screenRanking.controller, ...
    config.candidateControllerIds), :);
candidateRows = candidateRows(isfinite(candidateRows.weighted_total_error_ratio), :);
if height(candidateRows) < 2
    error('run_15_disturbance_control_comparison:InsufficientFinalists', ...
        'Fewer than two candidate controllers completed screening.');
end
finalistIds = candidateRows.controller(1:2);

fprintf('\n===== Stage 2c: finalist holdout/full exact matrix =====\n');
finalistControllers = candidateControllers(ismember( ...
    string({candidateControllers.id}), finalistIds));
[finalistRuns, scheduleCache] = runCases(finalistControllers, allCases, ...
    nominalReference, scheduleCache, setup, strictConfig, exactConfig); %#ok<ASGLU>
exactRuns = [baselineRuns; finalistRuns];
exactPerCase = summarizeDisturbanceRuns(exactRuns, "exact_validation");
exactRanking = rankDisturbanceControllers(exactPerCase, config);

simscape = struct('executed', false, 'runs', struct([]), ...
    'perCase', table(), 'ranking', table());
if config.runSimscapeFinalists
    fprintf('\n===== Stage 3: matched Simscape matrix =====\n');
    simscapeControllers = [baselineControllers; finalistControllers];
    simscapeCases = allCases;
    if ~isempty(config.simscapeCaseIds)
        simscapeCases = simscapeCases(ismember([simscapeCases.id], ...
            string(config.simscapeCaseIds)));
    end
    simscape = runSimscapeDisturbanceMatrix(config.trajectoryFile, ...
        simscapeControllers, simscapeCases, setup.model, setup.scene, ...
        setup.config, strictConfig, config);
end

if simscape.executed
    finalPerCase = simscape.perCase;
    finalRanking = simscape.ranking;
    conclusion = conclusionText(finalRanking, "Simscape");
else
    finalPerCase = exactPerCase;
    finalRanking = exactRanking;
    conclusion = conclusionText(finalRanking, "exact nonlinear validation");
end

result = struct();
result.method = "Matched disturbance-control comparison";
result.generatedAt = datetime('now', 'TimeZone', 'Asia/Shanghai');
result.config = config;
result.trajectoryFile = string(setup.trajectoryFile);
result.caseMatrix = allCases;
result.dobTuning = tuning;
result.bestDobConfig = bestDob;
result.screeningPerCase = screenTable;
result.screeningRanking = screenRanking;
result.finalistControllerIds = finalistIds;
result.exactRuns = exactRuns;
result.exactPerCase = exactPerCase;
result.exactRanking = exactRanking;
result.simscape = simscape;
result.perCase = finalPerCase;
result.ranking = finalRanking;
result.conclusion = conclusion;
result.evidenceBoundary = [ ...
    "Exact nonlinear screening is not labeled Simscape evidence.", ...
    "Ideal PIDF and pose-to-length cascade are appendix-only actuator-boundary references.", ...
    "The source SLX is modified in memory only and closed without saving."];
if config.writeArtifacts
    result.artifactPaths = exportDisturbanceComparisonEvidence(result, ...
        config.outputDir, config.evidenceDir, config);
end
clear cleanup;
end

function reference = makeReference(refs)
reference = struct('time', refs.t, 'q', refs.q, 'qd', refs.qd, ...
    'qdd', refs.qdd, 'computedForce', refs.Fcomputed, ...
    'legLength', refs.L, 'legSpeed', refs.Ld, ...
    'targetPerturbation', zeros(size(refs.q)), ...
    'feedforwardPolicy', "nominal");
end

function controller = makeController(id, schedule, useDob, useStrictQp, dobConfig, filterScene)
displayNames = struct('fixed_lqi', "Fixed computed-torque LQI", ...
    'strict_qp', "Current strict CLF-CBF-QP", ...
    'scheduled_lqi', "Reference-scheduled LQI", ...
    'fixed_lqi_dob', "Fixed LQI + DOB", ...
    'scheduled_lqi_dob', "Scheduled LQI + DOB", ...
    'scheduled_lqi_dob_strict_qp', "Scheduled LQI + DOB + strict QP");
controller = struct('id', string(id), 'displayName', displayNames.(char(id)), ...
    'schedule', schedule, 'useDob', logical(useDob), ...
    'useStrictQp', logical(useStrictQp), 'dobConfig', dobConfig, ...
    'filterScene', filterScene);
end

function [runs, scheduleCache] = runCases(controllers, cases, nominalReference, ...
        scheduleCache, setup, strictConfig, config)
runsCell = cell(numel(controllers)*numel(cases), 1);
runIndex = 0;
for caseIndex = 1:numel(cases)
    experimentCase = cases(caseIndex);
    reference = buildDisturbedReference(nominalReference, experimentCase, ...
        setup.model, setup.config, config);
    [scheduled, scheduleCache] = scheduleForCase(reference, experimentCase, ...
        scheduleCache, setup, config);
    for controllerIndex = 1:numel(controllers)
        runIndex = runIndex+1;
        controller = controllers(controllerIndex);
        if startsWith(controller.id, "scheduled")
            controller.schedule = scheduled;
        end
        fprintf('[%d/%d] %s | %s\n', runIndex, numel(runsCell), ...
            controller.id, experimentCase.id);
        runsCell{runIndex} = simulateDisturbanceControlScenario(reference, ...
            setup.model, setup.scene, setup.config, strictConfig, ...
            experimentCase, controller, config);
    end
end
runs = vertcat(runsCell{:});
end

function [schedule, cache] = scheduleForCase(reference, experimentCase, cache, setup, config)
if ~experimentCase.hasSmoothBump
    schedule = cache.nominal;
    return;
end
key = matlab.lang.makeValidName(char("bump_"+ ...
    replace(compose('%.1f', experimentCase.scale), '.', 'p')));
if isfield(cache, key)
    schedule = cache.(key);
    return;
end
scheduleReference = reference;
schedule = buildReferenceScheduledLqi(scheduleReference, setup.model, ...
    setup.config, 'mode', 'scheduled', 'stateStep', config.stateStep, ...
    'inputStep', config.inputStep);
cache.(key) = schedule;
end

function selected = selectCases(runs, ids)
mask = false(numel(runs), 1);
for index = 1:numel(runs)
    mask(index) = ismember(runs(index).experimentCase.id, ids);
end
selected = runs(mask);
end

function [bestConfig, tuning] = tuneDob(trainingCases, baselineRuns, setup, config)
grid = buildDobTuningGrid(config);
baselineTraining = selectCases(baselineRuns, [trainingCases.id]);
baselineTraining = baselineTraining(arrayfun( ...
    @(run) run.controller.id == "fixed_lqi", baselineTraining));
score = inf(height(grid), 1);
hardPassed = false(height(grid), 1);
maximumP95 = zeros(height(grid), 1);
for index = 1:height(grid)
    dob = makeDobConfig(config.sampleTime, grid.cutoff_hz(index), ...
        grid.leg_force_limit_n(index), grid.rate_limit_nps(index));
    [score(index), hardPassed(index), maximumP95(index)] = ...
        scoreDobReplay(dob, baselineTraining, setup.model);
    fprintf('DOB %s: residual ratio=%.6g, hard=%d, p95=%.6g s\n', ...
        char(string(grid.id(index))), score(index), hardPassed(index), maximumP95(index));
end
tuning = addvars(grid, score, hardPassed, maximumP95, ...
    'NewVariableNames', {'weighted_error_ratio', 'hard_passed', ...
    'maximum_controller_p95_s'});
eligible = hardPassed & isfinite(score);
if ~any(eligible)
    error('run_15_disturbance_control_comparison:NoDobConfiguration', ...
        'Every DOB grid point failed the exact-model training screen.');
end
eligibleIndices = find(eligible);
[~, local] = min(score(eligible));
bestIndex = eligibleIndices(local);
tuning.selected = false(height(tuning), 1);
tuning.selected(bestIndex) = true;
bestConfig = makeDobConfig(config.sampleTime, grid.cutoff_hz(bestIndex), ...
    grid.leg_force_limit_n(bestIndex), grid.rate_limit_nps(bestIndex));
bestConfig.id = string(grid.id(bestIndex));
end

function [score, passed, p95] = scoreDobReplay(dobConfig, runs, model)
caseScores = inf(numel(runs), 1);
stepTimes = [];
passed = true;
for runIndex = 1:numel(runs)
    run = runs(runIndex);
    state = initializeDisturbanceObserver();
    sampleCount = numel(run.control.time);
    residualNorm = zeros(1, sampleCount);
    disturbanceNorm = vecnorm(run.control.actualWrench, 2, 1);
    previousCompensation = zeros(6, 1);
    for sampleIndex = 1:sampleCount
        q = run.control.state(1:6, sampleIndex);
        qd = run.control.state(7:12, sampleIndex);
        timer = tic;
        [compensation, state] = stepDisturbanceObserver(q, qd, ...
            run.control.plantAcceleration(:, sampleIndex), ...
            run.control.appliedForce(:, sampleIndex), model, dobConfig, state);
        stepTimes(end+1, 1) = toc(timer); %#ok<AGROW>
        mapped = sgpJacobian(q, model).Jv.'*compensation;
        residualNorm(sampleIndex) = norm(run.control.actualWrench(:, sampleIndex)+mapped);
        if any(abs(compensation) > dobConfig.legForceLimit+1e-9) || ...
                any(abs(compensation-previousCompensation) > ...
                dobConfig.rateLimit*dobConfig.sampleTime+1e-9)
            passed = false;
        end
        previousCompensation = compensation;
    end
    denominator = max(sqrt(mean(disturbanceNorm.^2)), 1e-9);
    caseScores(runIndex) = sqrt(mean(residualNorm.^2))/denominator;
end
finiteScores = caseScores(isfinite(caseScores));
score = mean(finiteScores);
if isempty(finiteScores), score = Inf; passed = false; end
p95 = localPercentile(stepTimes, 95);
passed = passed && all(isfinite(stepTimes));
end

function value = localPercentile(data, probability)
if isempty(data), value = NaN; return; end
data = sort(data(:));
position = 1+(numel(data)-1)*probability/100;
lower = floor(position); upper = ceil(position);
if lower == upper, value = data(lower); else
    value = data(lower)+(position-lower)*(data(upper)-data(lower));
end
end

function warmStrictQp(reference, setup, filterScene, strictConfig)
previous = reference.computedForce(:, 1);
strictConfig.time = reference.time(1);
for index = 1:3
    stepStrictClfCbfQp(reference.q(:, 1), reference.qd(:, 1), ...
        reference.q(:, 1), reference.qd(:, 1), reference.qdd(:, 1), ...
        previous, previous, setup.model, filterScene, strictConfig);
end
end

function text = conclusionText(ranking, source)
accepted = ranking(ranking.better_than_existing, :);
if isempty(accepted)
    text = "No controller met every 'better than existing control' gate in "+ ...
        source+". The ranked errors remain useful, but no superiority claim is made.";
else
    best = accepted(1, :);
    text = sprintf(['%s is the best accepted controller in %s: aggregate ' ...
        'weighted error improved by %.2f%% with all declared gates passed.'], ...
        best.controller, source, 100*best.improvement_fraction);
end
end

function closeWithoutSaving(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end

function ensureLocalCasadiRuntime(projectRoot)
casadiCandidates = dir(fullfile(projectRoot, 'lib', ...
    'casadi-windows-matlabR*-v*'));
if isempty(casadiCandidates)
    error('run_15_disturbance_control_comparison:MissingCasadi', ...
        'The project-local CasADi runtime directory is missing.');
end
casadiRoot = fullfile(casadiCandidates(1).folder, casadiCandidates(1).name);
mexFile = fullfile(casadiRoot, 'casadiMEX.mexw64');
if ~ismember(exist(mexFile, 'file'), [2, 3])
    error('run_15_disturbance_control_comparison:MissingCasadiMex', ...
        ['The ignored CasADi MEX runtime is absent from this worktree. ' ...
        'Copy the local lib runtime before executing the experiment.']);
end
hslCandidates = dir(fullfile(projectRoot, 'lib', 'CoinHSL-*'));
runtimePath = casadiRoot;
if ~isempty(hslCandidates)
    hslBin = fullfile(hslCandidates(1).folder, hslCandidates(1).name, 'bin');
    if exist(hslBin, 'dir') == 7
        runtimePath = [runtimePath, pathsep, hslBin];
    end
end
addpath(casadiRoot, '-begin');
setenv('PATH', [runtimePath, pathsep, getenv('PATH')]);
try
    probe = casadi.SX.sym('disturbance_comparison_probe'); %#ok<NASGU>
catch exception
    error('run_15_disturbance_control_comparison:CasadiLoadFailed', ...
        'The project-local CasADi runtime could not be loaded: %s', ...
        exception.message);
end
end
