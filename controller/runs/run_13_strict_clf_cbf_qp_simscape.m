function result = run_13_strict_clf_cbf_qp_simscape(trajectoryFile, outputDir, options)
% run_13_strict_clf_cbf_qp_simscape - Run matched and nonideal evidence paths.
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

settings = defaultSettings();
settings = mergeKnownFields(settings, options);
if ischar(settings.scenarios)
    settings.scenarios = string(settings.scenarios);
else
    settings.scenarios = string(settings.scenarios(:));
end
if isempty(settings.scenarios) || any(~ismember(settings.scenarios, ...
        ["ideal-force", "nonideal-force"]))
    error('run_13_strict_clf_cbf_qp_simscape:InvalidScenarios', ...
        'scenarios must contain ideal-force and/or nonideal-force.');
end
if ~isempty(settings.stopTime)
    validateattributes(settings.stopTime, {'double'}, {'scalar', 'positive', 'finite'});
end
if strlength(string(outputDir)) == 0
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outputDir = fullfile(projectRoot, 'results', 'reports', ...
        ['strict_clf_cbf_qp_simscape_', timestamp]);
end
outputDir = char(outputDir);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

runs = repmat(struct('scenario', "", 'setup', struct(), 'report', struct(), ...
    'resultFile', "", 'error', ""), numel(settings.scenarios), 1);
for index = 1:numel(settings.scenarios)
    scenario = settings.scenarios(index);
    fprintf('\n===== Strict CLF-CBF-QP Simscape: %s =====\n', scenario);
    prepareOptions = struct('actuatorMode', scenario, ...
        'sampleTime', settings.sampleTime, ...
        'baselineOverrides', settings.baselineOverrides, ...
        'strictOverrides', settings.strictOverrides, ...
        'plantPerturbation', settings.plantPerturbation);
    setup = prepareSimscapeStrictClfCbfQp(trajectoryFile, prepareOptions);
    cleanup = onCleanup(@() closeModelWithoutSaving(setup.modelName));
    stopTime = setup.refs.t(end);
    if ~isempty(settings.stopTime)
        stopTime = min(stopTime, settings.stopTime);
    end
    simulationOutput = sim(setup.modelName, ...
        'SimulationMode', 'normal', ...
        'StopTime', num2str(stopTime, 16), ...
        'ReturnWorkspaceOutputs', 'on');
    report = evaluateSimscapeStrictClfCbfQp( ...
        simulationOutput.get('simout'), setup);
    report.fullTrajectoryCompleted = abs(stopTime-setup.refs.t(end)) <= 1e-12;

    safeName = strrep(char(scenario), '-', '_');
    resultFile = fullfile(outputDir, ['strict_qp_', safeName, '.mat']);
    save(resultFile, 'setup', 'report', 'trajectoryFile', 'settings');
    exportRunFigure(fullfile(outputDir, ['strict_qp_', safeName, '.png']), report);
    writeRunSummary(fullfile(outputDir, ['strict_qp_', safeName, '.txt']), ...
        resultFile, report);
    fprintf('passed=%d, fallback=%d, online-step p95=%.6g s, min CBF residual=%.6g\n', ...
        report.passed, report.metrics.fallbackCount, ...
        report.metrics.controllerStepTime95, ...
        report.metrics.minimumCbfResidual);

    runs(index).scenario = scenario;
    runs(index).setup = setup;
    runs(index).report = report;
    runs(index).resultFile = string(resultFile);
    clear cleanup;
end

result = struct();
result.outputDir = string(outputDir);
result.runs = runs;
result.settings = settings;
result.allPassed = all(arrayfun(@(item) item.report.passed, runs));
if settings.failOnRejected && ~result.allPassed
    error('run_13_strict_clf_cbf_qp_simscape:AcceptanceFailed', ...
        'At least one requested strict-QP Simscape scenario was rejected.');
end
end

function settings = defaultSettings()
settings = struct();
settings.scenarios = ["ideal-force", "nonideal-force"];
settings.sampleTime = 0.01;
settings.stopTime = [];
settings.baselineOverrides = struct();
settings.strictOverrides = struct();
settings.plantPerturbation = struct();
settings.failOnRejected = false;
end

function target = mergeKnownFields(target, source)
fields = fieldnames(source);
for index = 1:numel(fields)
    name = fields{index};
    if ~isfield(target, name)
        error('run_13_strict_clf_cbf_qp_simscape:UnknownOption', ...
            'Unknown run option: %s.', name);
    end
    target.(name) = source.(name);
end
end

function exportRunFigure(fileName, report)
figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1200, 900]);
tiledlayout(4, 1, 'TileSpacing', 'compact');
nexttile;
plot(report.poseTime, report.poseError(:, 1:3)*1e3, 'LineWidth', 1);
grid on; ylabel('mm'); title('Translation tracking error');
nexttile;
plot(report.poseTime, rad2deg(report.poseError(:, 4:6)), 'LineWidth', 1);
grid on; ylabel('deg'); title('Rotation tracking error');
nexttile;
plot(report.diagnosticTime, report.diagnostics.minimumCbfResidual, 'LineWidth', 1.1);
hold on; yline(0, 'r--'); grid on; ylabel('residual');
title('Minimum hard-CBF residual');
nexttile;
plot(report.diagnosticTime, report.diagnostics.controllerStepTime*1e3, 'LineWidth', 1);
hold on; yline(10, 'r--'); grid on; ylabel('ms'); xlabel('Time (s)');
title('QP solution time');
exportgraphics(figureHandle, fileName, 'Resolution', 180);
close(figureHandle);
end

function writeRunSummary(fileName, resultFile, report)
fid = fopen(fileName, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Strict CLF-CBF-QP Simscape evidence\n');
fprintf(fid, 'resultFile: %s\n', resultFile);
fprintf(fid, 'validationClass: %s\n', report.validationClass);
fprintf(fid, 'strictGuaranteeApplicable: %d\n', report.strictGuaranteeApplicable);
fprintf(fid, 'passed: %d\n', report.passed);
writeNumericStruct(fid, report.metrics);
writeNumericStruct(fid, report.acceptance);
fprintf(fid, 'forceSignAvailable: %d\n', report.forceSignCheck.available);
fprintf(fid, 'forceSignPassed: %d\n', report.forceSignCheck.passed);
end

function writeNumericStruct(fid, values)
names = fieldnames(values);
for index = 1:numel(names)
    value = values.(names{index});
    if isnumeric(value) || islogical(value)
        fprintf(fid, '%s: %s\n', names{index}, mat2str(value, 12));
    end
end
end

function closeModelWithoutSaving(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
