projectRoot = 'D:\NPU\Matlab_code\stewart-simscape-disturbance-experiment';
cd(projectRoot);
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(projectRoot, 'controller', 'simscape_tracking'));
addpath(fullfile(projectRoot, 'controller', 'strict_clf_cbf_qp'));
addpath(fullfile(projectRoot, 'controller', 'exact_model_study'));
addpath(fullfile(projectRoot, 'controller', 'lqi_schedule_comparison'));
addpath(fullfile(projectRoot, 'controller', 'disturbance_control_comparison'));
casadiRoot = fullfile(projectRoot, 'lib', 'casadi-windows-matlabR2018b-v3.7.2');
hslBin = fullfile(projectRoot, 'lib', ...
    'CoinHSL-archive.v2023.11.17.x86_64-w64-mingw32-libgfortran5', 'bin');
addpath(casadiRoot, '-begin');
setenv('PATH', [casadiRoot, pathsep, hslBin, pathsep, getenv('PATH')]);

matFile = fullfile(projectRoot, 'results', 'reports', ...
    'disturbance_control_comparison', 'disturbance_control_comparison.mat');
load(matFile, 'result');
config = result.config;
overrides = config.baselineOverrides;
overrides.controlLaw = 'computed-torque';
overrides.actuatorMode = 'ideal-force';
overrides.derivativeSampleTime = config.sampleTime;
setup = prepareSimscapePoseForceControl(config.trajectoryFile, overrides);
cleanup = onCleanup(@() closeModel(setup.modelName));
nominal = struct('time', setup.refs.t, 'q', setup.refs.q, ...
    'qd', setup.refs.qd, 'qdd', setup.refs.qdd, ...
    'computedForce', setup.refs.Fcomputed, 'legLength', setup.refs.L, ...
    'legSpeed', setup.refs.Ld, 'targetPerturbation', zeros(size(setup.refs.q)), ...
    'feedforwardPolicy', "nominal");
fixedSchedule = buildReferenceScheduledLqi(nominal, setup.model, setup.config, ...
    'mode', 'fixed', 'stateStep', config.stateStep, 'inputStep', config.inputStep);
scheduledSchedule = buildReferenceScheduledLqi(nominal, setup.model, setup.config, ...
    'mode', 'scheduled', 'stateStep', config.stateStep, 'inputStep', config.inputStep);
[filterScene, strictConfig] = makeDisturbanceStrictFilter( ...
    setup.model, setup.scene, config);
controllers = [makeController("strict_qp", fixedSchedule, false, ...
        config, result.bestDobConfig, filterScene); ...
    makeController("scheduled_lqi_dob_strict_qp", scheduledSchedule, true, ...
        config, result.bestDobConfig, filterScene)];
cases = result.caseMatrix;

exactCell = cell(numel(controllers)*numel(cases), 1);
runIndex = 0;
scheduleCache = struct();
for caseIndex = 1:numel(cases)
    reference = buildDisturbedReference(nominal, cases(caseIndex), ...
        setup.model, setup.config, config);
    schedule = scheduledSchedule;
    if cases(caseIndex).hasSmoothBump
        key = matlab.lang.makeValidName(char("bump_"+ ...
            replace(compose('%.1f', cases(caseIndex).scale), '.', 'p')));
        if ~isfield(scheduleCache, key)
            scheduleCache.(key) = buildReferenceScheduledLqi(reference, ...
                setup.model, setup.config, 'mode', 'scheduled', ...
                'stateStep', config.stateStep, 'inputStep', config.inputStep);
        end
        schedule = scheduleCache.(key);
    end
    for controllerIndex = 1:numel(controllers)
        runIndex = runIndex+1;
        controller = controllers(controllerIndex);
        if startsWith(controller.id, "scheduled"), controller.schedule = schedule; end
        fprintf('[strict exact rerun %d/%d] %s | %s\n', runIndex, ...
            numel(exactCell), controller.id, cases(caseIndex).id);
        exactCell{runIndex} = simulateDisturbanceControlScenario(reference, ...
            setup.model, setup.scene, setup.config, strictConfig, ...
            cases(caseIndex), controller, config);
    end
end
strictExactRuns = vertcat(exactCell{:});
set_param(setup.modelName, 'Dirty', 'off');
close_system(setup.modelName, 0);
clear cleanup;

strictSimscape = runSimscapeDisturbanceMatrix(config.trajectoryFile, ...
    controllers, cases, setup.model, setup.scene, setup.config, ...
    strictConfig, config);
result.exactRuns = replaceRuns(result.exactRuns, strictExactRuns, ...
    ["strict_qp", "scheduled_lqi_dob_strict_qp"]);
result.simscape.runs = replaceRuns(result.simscape.runs, ...
    strictSimscape.runs, ["strict_qp", "scheduled_lqi_dob_strict_qp"]);
result = refreshDisturbanceCollisionAssessment(result);
result.generatedAt = datetime('now', 'TimeZone', 'Asia/Shanghai');
result.artifactPaths = exportDisturbanceComparisonEvidence(result, ...
    result.config.outputDir, result.config.evidenceDir, result.config);
fprintf('\nSTRICT_FILTER_RERUN_COMPLETED\n');
disp(result.exactRanking);
disp(result.simscape.ranking);

function value = makeController(id, schedule, useDob, config, dobConfig, filterScene)
names = struct('strict_qp', "Current strict CLF-CBF-QP", ...
    'scheduled_lqi_dob_strict_qp', "Scheduled LQI + DOB + strict QP");
value = struct('id', string(id), 'displayName', names.(char(id)), ...
    'schedule', schedule, 'useDob', logical(useDob), 'useStrictQp', true, ...
    'dobConfig', dobConfig, 'filterScene', filterScene);
if ~useDob
    value.dobConfig = makeDobConfig(config.sampleTime, 5, 120, 2000);
end
end

function output = replaceRuns(existing, replacement, ids)
keep = true(numel(existing), 1);
for index = 1:numel(existing)
    keep(index) = ~ismember(existing(index).controller.id, ids);
end
output = [existing(keep); replacement];
end

function closeModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
