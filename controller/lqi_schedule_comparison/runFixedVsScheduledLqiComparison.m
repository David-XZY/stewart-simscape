function result = runFixedVsScheduledLqiComparison(trajectoryFile, outputDir, options)
% runFixedVsScheduledLqiComparison - Reproducible fixed/scheduled LQI study.
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
addpath(runRoot);

settings = mergeKnownFields(defaultSettings(), options);
if strlength(string(outputDir)) == 0
    outputDir = fullfile(projectRoot, 'results', 'reports', ...
        'lqi_schedule_comparison');
end
outputDir = char(outputDir);
if settings.writeArtifacts && ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

setup = prepareSimscapePoseForceControl(trajectoryFile, settings.baselineOverrides);
cleanup = onCleanup(@() closePreparedModel(setup.modelName));
reference = makeReference(setup.refs, settings.stopTime);
fixedSchedule = buildReferenceScheduledLqi(reference, setup.model, setup.config, ...
    'mode', 'fixed', 'stateStep', settings.stateStep, ...
    'inputStep', settings.inputStep);
scheduledSchedule = buildReferenceScheduledLqi(reference, setup.model, setup.config, ...
    'mode', 'scheduled', 'stateStep', settings.stateStep, ...
    'inputStep', settings.inputStep);

scenarios = buildScenarios(setup.model, settings);
controllers = struct('id', {"fixed_lqi", "scheduled_lqi"}, ...
    'displayName', {"Fixed LQI", "Reference-scheduled LQI"}, ...
    'schedule', {fixedSchedule, scheduledSchedule});
runsCell = cell(numel(controllers)*numel(scenarios), 1);
runIndex = 0;
for controllerIndex = 1:numel(controllers)
    for scenarioIndex = 1:numel(scenarios)
        runIndex = runIndex+1;
        run = simulateLqiScheduleScenario(reference, controllers(controllerIndex).schedule, ...
            setup.config, scenarios(scenarioIndex), ...
            'integrationSubsteps', settings.integrationSubsteps, ...
            'initialPoseOffset', settings.initialPoseOffset, ...
            'initialVelocityOffset', settings.initialVelocityOffset);
        run.controller = rmfield(controllers(controllerIndex), 'schedule');
        runsCell{runIndex} = run;
    end
end
runs = vertcat(runsCell{:});

summary = summarizeRuns(runs);
result = struct();
result.method = "Fixed versus reference-scheduled LQI exact nonlinear study";
result.generatedAt = datetime('now', 'TimeZone', 'Asia/Shanghai');
result.trajectoryFile = string(setup.trajectoryFile);
result.reference = reference;
result.settings = settings;
result.fixedSchedule = fixedSchedule;
result.scheduledSchedule = scheduledSchedule;
result.runs = runs;
result.summary = summary;
result.conclusionAssessment = assessComparison(runs);
result.scope = ["The study compares only computed-torque plus LQI nominal control.", ...
    "The scheduled controller is precomputed from the known reference trajectory.", ...
    "No QP, online relinearization, global-stability claim, or constraint guarantee is included."];

artifactPaths = struct();
if settings.writeArtifacts
    artifactPaths.summaryCsv = string(fullfile(outputDir, 'fixed_vs_scheduled_lqi_summary.csv'));
    artifactPaths.matFile = string(fullfile(outputDir, 'fixed_vs_scheduled_lqi_result.mat'));
    writetable(summary, artifactPaths.summaryCsv);
end

if settings.exportFigures
    artifactPaths.figures = exportFigures(runs, outputDir, settings.figureResolution);
end
result.artifactPaths = artifactPaths;
if settings.writeArtifacts
    save(artifactPaths.matFile, 'result', '-v7.3');
end
clear cleanup;
end

function assessment = assessComparison(runs)
controllerIds = strings(numel(runs), 1);
runScenarioIds = strings(numel(runs), 1);
for itemIndex = 1:numel(runs)
    controllerIds(itemIndex) = runs(itemIndex).controller.id;
    runScenarioIds(itemIndex) = runs(itemIndex).scenario.id;
end
scenarioIds = unique(runScenarioIds, 'stable');
scenarioIds = scenarioIds(:);
count = numel(scenarioIds);
translationImproved = false(count, 1);
rotationImproved = false(count, 1);
forceRiskNotIncreased = false(count, 1);
noNewPhysicalViolation = false(count, 1);
bothCompleted = false(count, 1);
for index = 1:count
    rows = runs(controllerIds == "fixed_lqi" & ...
        runScenarioIds == scenarioIds(index));
    fixed = rows(1).metrics;
    rows = runs(controllerIds == "scheduled_lqi" & ...
        runScenarioIds == scenarioIds(index));
    scheduled = rows(1).metrics;
    translationImproved(index) = scheduled.translationRmse < fixed.translationRmse;
    rotationImproved(index) = scheduled.rotationRmse < fixed.rotationRmse;
    forceRiskNotIncreased(index) = ...
        scheduled.peakCommandForce <= fixed.peakCommandForce+1e-9 && ...
        scheduled.peakCommandForceRate <= fixed.peakCommandForceRate+1e-9;
    fixedViolations = fixed.lengthViolationCount+fixed.speedViolationCount+ ...
        fixed.accelerationViolationCount+fixed.forceViolationCount;
    scheduledViolations = scheduled.lengthViolationCount+scheduled.speedViolationCount+ ...
        scheduled.accelerationViolationCount+scheduled.forceViolationCount;
    noNewPhysicalViolation(index) = scheduledViolations <= fixedViolations;
    bothCompleted(index) = fixed.fullTrajectoryCompleted && scheduled.fullTrajectoryCompleted;
end
assessment = table(scenarioIds, translationImproved, rotationImproved, ...
    forceRiskNotIncreased, noNewPhysicalViolation, bothCompleted, ...
    'VariableNames', {'scenario', 'translation_rmse_improved', ...
    'rotation_rmse_improved', 'force_risk_not_increased', ...
    'no_new_physical_violation', 'both_full_trajectory_completed'});
end

function settings = defaultSettings()
settings = struct();
settings.stopTime = [];
settings.integrationSubsteps = 2;
settings.initialPoseOffset = [0.0004; -0.0003; 0.0002; 0.0002; -0.00015; 0.0001];
settings.initialVelocityOffset = zeros(6, 1);
settings.massScale = 1.20;
settings.comOffset = [10; -8; 5] * 1e-3;
settings.actuatorLag = 0.030;
settings.stateStep = [1e-6 * ones(6, 1); 1e-5 * ones(6, 1)];
settings.inputStep = 1;
settings.baselineOverrides = struct();
settings.writeArtifacts = true;
settings.exportFigures = true;
settings.figureResolution = 220;
end

function target = mergeKnownFields(target, source)
names = fieldnames(source);
for index = 1:numel(names)
    if ~isfield(target, names{index})
        error('runFixedVsScheduledLqiComparison:UnknownOption', ...
            'Unknown option: %s.', names{index});
    end
    target.(names{index}) = source.(names{index});
end
end

function reference = makeReference(refs, stopTime)
lastIndex = numel(refs.t);
if ~isempty(stopTime)
    valid = find(refs.t <= stopTime+1e-12);
    if numel(valid) < 2
        error('runFixedVsScheduledLqiComparison:StopTimeTooShort', ...
            'stopTime must retain at least two samples.');
    end
    lastIndex = valid(end);
end
indices = 1:lastIndex;
reference = struct('time', refs.t(indices), 'q', refs.q(:, indices), ...
    'qd', refs.qd(:, indices), 'qdd', refs.qdd(:, indices), ...
    'computedForce', refs.Fcomputed(:, indices));
end

function scenarios = buildScenarios(nominalModel, settings)
massModel = nominalModel;
massModel.dynamics.totalMass = settings.massScale * nominalModel.dynamics.totalMass;
massModel.dynamics.inertiaAtCOM_P = settings.massScale * nominalModel.dynamics.inertiaAtCOM_P;
platformMass = nominalModel.dynamics.totalMass-nominalModel.dynamics.objectMass;
massModel.dynamics.objectMass = massModel.dynamics.totalMass-platformMass;
comModel = nominalModel;
comModel.dynamics.comP = nominalModel.dynamics.comP+settings.comOffset;
template = struct('id', "", 'displayName', "", 'plantModel', nominalModel, ...
    'actuatorLag', 0);
scenarios = repmat(template, 4, 1);
scenarios(1) = makeScenario(template, "nominal", "Nominal model", nominalModel, 0);
scenarios(2) = makeScenario(template, "mass_120", "Mass and inertia +20%", massModel, 0);
scenarios(3) = makeScenario(template, "com_offset", "COM offset [10 -8 5] mm", comModel, 0);
scenarios(4) = makeScenario(template, "lag_30ms", "First-order actuator lag 30 ms", ...
    nominalModel, settings.actuatorLag);
end

function value = makeScenario(template, id, displayName, model, actuatorLag)
value = template;
value.id = string(id);
value.displayName = string(displayName);
value.plantModel = model;
value.actuatorLag = actuatorLag;
end

function summary = summarizeRuns(runs)
count = numel(runs);
controller = strings(count, 1);
scenario = strings(count, 1);
values = zeros(count, 15);
for index = 1:count
    controller(index) = runs(index).controller.id;
    scenario(index) = runs(index).scenario.id;
    m = runs(index).metrics;
    values(index, :) = [m.translationRmse, m.rotationRmse, m.translationPeak, ...
        m.rotationPeak, m.peakLqiFeedbackForce, m.peakCommandForce, ...
        m.peakAppliedForce, m.controlEnergy, m.peakCommandForceRate, ...
        m.minSigma, m.lengthViolationCount, m.speedViolationCount, ...
        m.accelerationViolationCount, m.forceViolationCount, ...
        m.fullTrajectoryCompleted];
end
summary = array2table(values, 'VariableNames', {'translation_rmse', ...
    'rotation_rmse', 'translation_peak', 'rotation_peak', ...
    'peak_lqi_feedback_force', 'peak_command_force', 'peak_applied_force', ...
    'control_energy', 'peak_command_force_rate', 'min_sigma', ...
    'length_violation_count', 'speed_violation_count', ...
    'acceleration_violation_count', 'force_violation_count', ...
    'full_trajectory_completed'});
summary = addvars(summary, controller, scenario, 'Before', 1);
end

function paths = exportFigures(runs, outputDir, resolution)
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
runScenarioIds = strings(numel(runs), 1);
for runIndex = 1:numel(runs)
    runScenarioIds(runIndex) = runs(runIndex).scenario.id;
end
scenarioNames = unique(runScenarioIds, 'stable');
figureOne = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1300 250*numel(scenarioNames)]);
tiledlayout(numel(scenarioNames), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
figureTwo = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1300 250*numel(scenarioNames)]);
tiledlayout(numel(scenarioNames), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for scenarioIndex = 1:numel(scenarioNames)
    matching = runs(runScenarioIds == scenarioNames(scenarioIndex));
    figure(figureOne);
    nexttile;
    hold on;
    for index = 1:numel(matching)
        errorNorm = vecnorm(matching(index).control.poseError(1:3, :), 2, 1);
        plot(matching(index).control.time, errorNorm, 'DisplayName', matching(index).controller.displayName);
    end
    title(sprintf('%s: translation error', scenarioNames(scenarioIndex)));
    ylabel('m'); grid on; legend('Location', 'best');
    nexttile;
    hold on;
    for index = 1:numel(matching)
        errorNorm = vecnorm(matching(index).control.poseError(4:6, :), 2, 1);
        plot(matching(index).control.time, errorNorm, 'DisplayName', matching(index).controller.displayName);
    end
    title(sprintf('%s: rotation error', scenarioNames(scenarioIndex)));
    ylabel('rad'); grid on; legend('Location', 'best');

    figure(figureTwo);
    nexttile;
    hold on;
    for index = 1:numel(matching)
        feedbackNorm = vecnorm(matching(index).control.feedbackForce, 2, 1);
        totalNorm = vecnorm(matching(index).control.commandForce, 2, 1);
        plot(matching(index).control.time, feedbackNorm, '-', 'DisplayName', ...
            matching(index).controller.displayName+" feedback");
        plot(matching(index).control.time, totalNorm, '--', 'DisplayName', ...
            matching(index).controller.displayName+" total");
    end
    title(sprintf('%s: feedback and total force', scenarioNames(scenarioIndex)));
    ylabel('N'); grid on; legend('Location', 'best');
    nexttile;
    hold on;
    for index = 1:numel(matching)
        plot(matching(index).dense.time, matching(index).dense.sigmaMin, ...
            'DisplayName', matching(index).controller.displayName);
    end
    title(sprintf('%s: minimum singular value', scenarioNames(scenarioIndex)));
    ylabel('\sigma_{min}'); grid on; legend('Location', 'best');
end
paths = struct();
paths.errorFigure = string(fullfile(outputDir, 'fixed_vs_scheduled_lqi_errors.png'));
paths.forceSigmaFigure = string(fullfile(outputDir, 'fixed_vs_scheduled_lqi_force_sigma.png'));
exportgraphics(figureOne, paths.errorFigure, 'Resolution', resolution);
exportgraphics(figureTwo, paths.forceSigmaFigure, 'Resolution', resolution);
close(figureOne);
close(figureTwo);
end

function closePreparedModel(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
