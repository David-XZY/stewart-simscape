function result = run_16_lqi_aw_command_governor_ablation(options)
% run_16_lqi_aw_command_governor_ablation - Matched 2-by-2 controller study.
arguments
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
addpath(fullfile(controllerRoot, 'lqi_schedule_comparison'));
addpath(fullfile(controllerRoot, 'disturbance_control_comparison'));
ensureLocalCasadiRuntime(projectRoot);

defaults = struct();
defaults.caseIds = ["nominal", "wrench_smooth_target_x0p5", ...
    "target_noise_x1p0_s101", "target_noise_x1p5_s102", ...
    "wrench_target_noise_x1p5_s102"];
defaults.runSimscape = true;
defaults.writeArtifacts = true;
defaults.failOnRejected = true;
defaults.controllerIds = ["scheduled_lqi_dob_strict_qp", ...
    "scheduled_lqi_dob_strict_qp_aw", ...
    "scheduled_lqi_dob_strict_qp_governor", ...
    "scheduled_lqi_dob_strict_qp_aw_governor"];
defaults.commandGovernorCutoffHz = 2.0;
defaults.qpAwareAntiWindupMismatchLimit = 30;
defaults.qpAwareAntiWindupGain = 0.1;
defaults.outputDir = fullfile(projectRoot, 'results', 'reports', ...
    'lqi_aw_command_governor_ablation');
defaults.evidenceDir = fullfile(projectRoot, 'docs', 'experiments', ...
    'disturbance-control-comparison', 'lqi-aw-command-governor');
options = mergeKnown(defaults, options);
options.caseIds = string(options.caseIds);
options.controllerIds = string(options.controllerIds);

config = makeDisturbanceComparisonConfig(struct( ...
    'writeArtifacts', false, 'exportFigures', false, ...
    'failOnRejected', options.failOnRejected, ...
    'commandGovernorCutoffHz', options.commandGovernorCutoffHz, ...
    'qpAwareAntiWindupMismatchLimit', ...
    options.qpAwareAntiWindupMismatchLimit, ...
    'qpAwareAntiWindupGain', options.qpAwareAntiWindupGain));
baselineOverrides = config.baselineOverrides;
baselineOverrides.controlLaw = 'computed-torque';
baselineOverrides.actuatorMode = 'ideal-force';
baselineOverrides.derivativeSampleTime = config.sampleTime;
setup = prepareSimscapePoseForceControl(config.trajectoryFile, baselineOverrides);
cleanup = onCleanup(@() closeWithoutSaving(setup.modelName));
nominal = makeReference(setup.refs);
schedule = buildReferenceScheduledLqi(nominal, setup.model, setup.config, ...
    'mode', 'scheduled', 'stateStep', config.stateStep, ...
    'inputStep', config.inputStep);
[filterScene, strictConfig] = makeDisturbanceStrictFilter( ...
    setup.model, setup.scene, config);
dob = makeDobConfig(config.sampleTime, 8, 180, 2000);
controllers = makeAblationControllers(schedule, dob, filterScene, config);
controllers = controllers(ismember([controllers.id], options.controllerIds));
if isempty(controllers) || ~any([controllers.id] == ...
        "scheduled_lqi_dob_strict_qp")
    error('run_16:MissingBaseline', ...
        'controllerIds must include scheduled_lqi_dob_strict_qp.');
end
allCases = buildDisturbanceCaseMatrix(config, 'all');
cases = allCases(ismember([allCases.id], string(options.caseIds)));
if numel(cases) ~= numel(options.caseIds)
    error('run_16_lqi_aw_command_governor_ablation:UnknownCase', ...
        'At least one requested case identifier was not found.');
end

exactRuns = runExactAblation(controllers, cases, nominal, setup, ...
    strictConfig, config);
exactPerCase = summarizeDisturbanceRuns(exactRuns, "exact_ablation");
exactAggregate = aggregateAblation(exactPerCase, controllers(1).id);

simscape = struct('executed', false, 'runs', struct([]), ...
    'perCase', table(), 'aggregate', table());
if options.runSimscape
    model = setup.model;
    scene = setup.scene;
    poseConfig = setup.config;
    clear cleanup;
    matrix = runSimscapeDisturbanceMatrix(config.trajectoryFile, controllers, ...
        cases, model, scene, poseConfig, strictConfig, config);
    simscape = matrix;
    simscape.aggregate = aggregateAblation( ...
        matrix.perCase, controllers(1).id);
end

result = struct('method', "QP-aware LQI anti-windup and command-governor ablation", ...
    'generatedAt', datetime('now', 'TimeZone', 'Asia/Shanghai'), ...
    'config', config, 'options', options, 'controllers', controllers, ...
    'cases', cases, 'exactRuns', exactRuns, 'exactPerCase', exactPerCase, ...
    'exactAggregate', exactAggregate, 'simscape', simscape);
if options.writeArtifacts
    result.artifactPaths = exportArtifacts(result, options);
end
clear cleanup;
end

function reference = makeReference(refs)
reference = struct('time', refs.t, 'q', refs.q, 'qd', refs.qd, ...
    'qdd', refs.qdd, 'computedForce', refs.Fcomputed, ...
    'legLength', refs.L, 'legSpeed', refs.Ld, ...
    'targetPerturbation', zeros(size(refs.q)), ...
    'commandNoisePerturbation', zeros(size(refs.q)), ...
    'feedforwardPolicy', "nominal");
end

function controllers = makeAblationControllers(schedule, dob, scene, config)
ids = ["scheduled_lqi_dob_strict_qp", ...
    "scheduled_lqi_dob_strict_qp_aw", ...
    "scheduled_lqi_dob_strict_qp_governor", ...
    "scheduled_lqi_dob_strict_qp_aw_governor"];
names = ["DOB-aware strict QP baseline", ...
    "Baseline + QP-aware LQI anti-windup", ...
    "Baseline + attitude command governor", ...
    "Baseline + anti-windup + command governor"];
useAw = [false, true, false, true];
useGovernor = [false, false, true, true];
template = struct('id', "", 'displayName', "", 'schedule', schedule, ...
    'useDob', true, 'useStrictQp', true, 'dobConfig', dob, ...
    'filterScene', scene, 'useQpAwareAntiWindup', false, ...
    'qpAwareAntiWindupMismatchLimit', ...
    config.qpAwareAntiWindupMismatchLimit, ...
    'qpAwareAntiWindupGain', config.qpAwareAntiWindupGain, ...
    'useCommandGovernor', false);
controllers = repmat(template, numel(ids), 1);
for index = 1:numel(ids)
    controllers(index).id = ids(index);
    controllers(index).displayName = names(index);
    controllers(index).useQpAwareAntiWindup = useAw(index);
    controllers(index).useCommandGovernor = useGovernor(index);
end
end

function runs = runExactAblation(controllers, cases, nominal, setup, strict, config)
runsCell = cell(numel(controllers)*numel(cases), 1);
runIndex = 0;
scheduleCache = struct();
for caseIndex = 1:numel(cases)
    experimentCase = cases(caseIndex);
    reference = buildDisturbedReference(nominal, experimentCase, ...
        setup.model, setup.config, config);
    if experimentCase.hasSmoothBump
        key = matlab.lang.makeValidName(char("bump_"+ ...
            replace(compose('%.1f', experimentCase.scale), '.', 'p')));
        if ~isfield(scheduleCache, key)
            scheduleReference = reference;
            scheduleReference.q = scheduleReference.q- ...
                scheduleReference.commandNoisePerturbation;
            scheduleCache.(key) = buildReferenceScheduledLqi(scheduleReference, ...
                setup.model, setup.config, 'mode', 'scheduled', ...
                'stateStep', config.stateStep, 'inputStep', config.inputStep);
        end
        activeSchedule = scheduleCache.(key);
    else
        activeSchedule = controllers(1).schedule;
    end
    for controllerIndex = 1:numel(controllers)
        runIndex = runIndex+1;
        controller = controllers(controllerIndex);
        controller.schedule = activeSchedule;
        fprintf('[Exact ablation %d/%d] %s | %s\n', runIndex, ...
            numel(runsCell), controller.id, experimentCase.id);
        runsCell{runIndex} = simulateDisturbanceControlScenario(reference, ...
            setup.model, setup.scene, setup.config, strict, experimentCase, ...
            controller, config);
    end
end
runs = vertcat(runsCell{:});
end

function aggregate = aggregateAblation(perCase, baselineId)
controllers = unique(perCase.controller, 'stable');
baseline = perCase(perCase.controller == baselineId, :);
ratio = nan(numel(controllers), 1);
rmsRatio = ratio;
peakRatio = ratio;
fallbacks = zeros(numel(controllers), 1);
infeasible = fallbacks;
accelerationCases = fallbacks;
collisionCases = fallbacks;
maximumP95 = ratio;
allEligible = false(numel(controllers), 1);
for index = 1:numel(controllers)
    rows = perCase(perCase.controller == controllers(index), :);
    [common, rowIndex, baseIndex] = intersect( ...
        rows.case_id, baseline.case_id, 'stable');
    if numel(common) ~= height(rows), continue; end
    rmsValues = rows.equivalent_pose_rms(rowIndex)./ ...
        max(baseline.equivalent_pose_rms(baseIndex), eps);
    peakValues = rows.equivalent_pose_peak(rowIndex)./ ...
        max(baseline.equivalent_pose_peak(baseIndex), eps);
    rmsRatio(index) = mean(rmsValues);
    peakRatio(index) = mean(peakValues);
    ratio(index) = mean(0.7*rmsValues+0.3*peakValues);
    fallbacks(index) = sum(rows.fallback_count);
    infeasible(index) = sum(rows.infeasible_count);
    accelerationCases(index) = sum(rows.acceleration_violation_count > 0);
    collisionCases(index) = sum(rows.collision_violation_count > 0);
    maximumP95(index) = max(rows.controller_time_p95_s);
    allEligible(index) = all(logical(rows.eligible));
end
aggregate = table(controllers, ratio, rmsRatio, peakRatio, fallbacks, ...
    infeasible, accelerationCases, collisionCases, maximumP95, allEligible, ...
    'VariableNames', {'controller', 'weighted_error_ratio', ...
    'mean_rms_ratio', 'mean_peak_ratio', 'fallback_count', ...
    'infeasible_count', 'acceleration_violation_cases', ...
    'collision_violation_cases', 'maximum_p95_s', 'all_cases_eligible'});
aggregate = sortrows(aggregate, {'all_cases_eligible', 'weighted_error_ratio'}, ...
    {'descend', 'ascend'});
end

function paths = exportArtifacts(result, options)
if exist(options.outputDir, 'dir') ~= 7, mkdir(options.outputDir); end
if exist(options.evidenceDir, 'dir') ~= 7, mkdir(options.evidenceDir); end
paths = struct('matFile', string(fullfile(options.outputDir, 'ablation.mat')), ...
    'exactCsv', string(fullfile(options.evidenceDir, 'exact_per_case.csv')), ...
    'exactAggregateCsv', string(fullfile(options.evidenceDir, 'exact_aggregate.csv')), ...
    'simscapeCsv', string(fullfile(options.evidenceDir, 'simscape_per_case.csv')), ...
    'simscapeAggregateCsv', string(fullfile(options.evidenceDir, ...
    'simscape_aggregate.csv')), 'conclusion', string(fullfile( ...
    options.evidenceDir, 'conclusion.md')));
save(paths.matFile, 'result', '-v7.3');
writetable(result.exactPerCase, paths.exactCsv);
writetable(result.exactAggregate, paths.exactAggregateCsv);
if result.simscape.executed
    writetable(result.simscape.perCase, paths.simscapeCsv);
    writetable(result.simscape.aggregate, paths.simscapeAggregateCsv);
end
writeConclusion(paths.conclusion, result);
end

function writeConclusion(path, result)
fid = fopen(path, 'w');
if fid < 0, error('run_16:OpenFailed', 'Cannot write %s.', path); end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# LQI anti-windup and command-governor ablation\n\n');
fprintf(fid, 'Generated: %s\n\n', char(result.generatedAt));
fprintf(fid, ['Governor: %.3g Hz, QP-aware mismatch limit: %.3g N, ' ...
    'cross-layer gain: %.3g.\n\n'], ...
    result.config.commandGovernorCutoffHz, ...
    result.config.qpAwareAntiWindupMismatchLimit, ...
    result.config.qpAwareAntiWindupGain);
if result.simscape.executed
    tableValue = result.simscape.aggregate;
    fprintf(fid, '## Simscape aggregate\n\n');
else
    tableValue = result.exactAggregate;
    fprintf(fid, '## Exact-model aggregate\n\n');
end
fprintf(fid, '|Controller|Weighted ratio|Fallbacks|Acceleration cases|Collision cases|Max P95 (ms)|Eligible|\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|:---:|\n');
for index = 1:height(tableValue)
    row = tableValue(index, :);
    fprintf(fid, '|%s|%.6g|%d|%d|%d|%.4g|%d|\n', row.controller, ...
        row.weighted_error_ratio, row.fallback_count, ...
        row.acceleration_violation_cases, row.collision_violation_cases, ...
        1e3*row.maximum_p95_s, row.all_cases_eligible);
end
fprintf(fid, '\nThe baseline is the already DOB-aware scheduled-LQI+DOB+strict-QP path. ');
fprintf(fid, ['This is an ablation of two runtime features, not a replacement ' ...
    'for the complete 40-case acceptance matrix.\n']);
end

function target = mergeKnown(target, source)
names = fieldnames(source);
for index = 1:numel(names)
    name = names{index};
    if ~isfield(target, name)
        error('run_16:UnknownOption', 'Unknown option %s.', name);
    end
    target.(name) = source.(name);
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
    error('run_16:MissingCasadi', 'The project-local CasADi runtime is missing.');
end
casadiRoot = fullfile(casadiCandidates(1).folder, casadiCandidates(1).name);
if ~ismember(exist(fullfile(casadiRoot, 'casadiMEX.mexw64'), 'file'), [2, 3])
    error('run_16:MissingCasadiMex', 'The ignored CasADi MEX is missing.');
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
probe = casadi.SX.sym('aw_governor_probe'); %#ok<NASGU>
end
