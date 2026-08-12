function result = runSimscapeDisturbanceMatrix(trajectoryFile, controllers, cases, ...
        model, scene, poseConfig, strictConfig, config)
% runSimscapeDisturbanceMatrix - Execute finalists and baselines in one matrix.
% Every model edit is in memory; each run closes the model with save flag 0.
arguments
    trajectoryFile {mustBeTextScalar}
    controllers struct
    cases struct
    model struct
    scene struct
    poseConfig struct
    strictConfig struct
    config struct
end

if bdIsLoaded('stewart_platform_model')
    set_param('stewart_platform_model', 'Dirty', 'off');
    close_system('stewart_platform_model', 0);
end
runsCell = cell(numel(controllers)*numel(cases), 1);
runIndex = 0;
for controllerIndex = 1:numel(controllers)
    controller = controllers(controllerIndex);
    try
        [setup, nominal] = prepareController(trajectoryFile, controller, ...
            strictConfig, config);
        cleanup = onCleanup(@() closeWithoutSaving(setup.modelName));
        fastRestartActive = config.useFastRestart && ...
            ~startsWith(controller.id, "scheduled");
        if fastRestartActive
            set_param(setup.modelName, 'FastRestart', 'on');
        end
    catch exception
        if bdIsLoaded('stewart_platform_model')
            set_param('stewart_platform_model', 'Dirty', 'off');
            close_system('stewart_platform_model', 0);
        end
        if config.failOnRejected, rethrow(exception); end
        for caseIndex = 1:numel(cases)
            runIndex = runIndex+1;
            runsCell{runIndex} = failedRun(controller, cases(caseIndex), exception);
        end
        continue;
    end
    scheduleCache = struct();
    for caseIndex = 1:numel(cases)
        experimentCase = cases(caseIndex);
        runIndex = runIndex+1;
        fprintf('[Simscape %d/%d] %s | %s\n', runIndex, numel(runsCell), ...
            controller.id, experimentCase.id);
        try
            reference = buildDisturbedReference(nominal, experimentCase, ...
                model, poseConfig, config);
            setup = applyRuntimeInputs(setup, reference, experimentCase, config);
            if isfield(setup, 'candidateRuntime')
                runtime = setup.candidateRuntime;
                if startsWith(controller.id, "scheduled") && experimentCase.hasSmoothBump
                    key = matlab.lang.makeValidName(char("bump_"+ ...
                        replace(compose('%.1f', experimentCase.scale), '.', 'p')));
                    if ~isfield(scheduleCache, key)
                        scheduleCache.(key) = buildReferenceScheduledLqi(reference, ...
                            setup.model, setup.config, 'mode', 'scheduled', ...
                            'stateStep', config.stateStep, 'inputStep', config.inputStep);
                    end
                    runtime.schedule = scheduleCache.(key);
                else
                    runtime.schedule = controller.schedule;
                end
                assignin('base', 'candidateControllerRuntime', runtime);
                setup.candidateRuntime = runtime;
            end
            simulationOutput = sim(setup.modelName, ...
                'StopTime', num2str(reference.time(end), 16), ...
                'ReturnWorkspaceOutputs', 'on');
            runsCell{runIndex} = evaluateSimscapeDisturbanceRun( ...
                simulationOutput.get('simout'), setup, reference, ...
                experimentCase, controller, strictConfig, scene, config);
        catch exception
            if config.failOnRejected
                rethrow(exception);
            end
            warning('runSimscapeDisturbanceMatrix:RunFailed', ...
                '%s | %s failed: %s', controller.id, experimentCase.id, ...
                exception.message);
            runsCell{runIndex} = failedRun(controller, experimentCase, exception);
        end
    end
    if fastRestartActive && bdIsLoaded(setup.modelName)
        set_param(setup.modelName, 'FastRestart', 'off');
    end
    clear cleanup;
end
runs = vertcat(runsCell{:});
perCase = summarizeDisturbanceRuns(runs, "simscape_validation");
if any(perCase.controller == "fixed_lqi")
    ranking = rankDisturbanceControllers(perCase, config);
else
    ranking = table();
end
result = struct('executed', true, 'runs', runs, ...
    'perCase', perCase, 'ranking', ranking);
end

function [setup, nominal] = prepareController(trajectoryFile, controller, ...
        strictConfig, config)
if controller.id == "strict_qp"
    options = struct('actuatorMode', "ideal-force", ...
        'sampleTime', config.sampleTime, ...
        'baselineOverrides', config.baselineOverrides, ...
        'strictOverrides', config.strictOverrides);
    setup = prepareSimscapeStrictClfCbfQp(trajectoryFile, options);
else
    overrides = config.baselineOverrides;
    overrides.controlLaw = 'computed-torque';
    overrides.actuatorMode = 'ideal-force';
    overrides.derivativeSampleTime = config.sampleTime;
    setup = prepareSimscapePoseForceControl(trajectoryFile, overrides);
end
nominal = struct('time', setup.refs.t, 'q', setup.refs.q, ...
    'qd', setup.refs.qd, 'qdd', setup.refs.qdd, ...
    'computedForce', setup.refs.Fcomputed, 'legLength', setup.refs.L, ...
    'legSpeed', setup.refs.Ld, 'targetPerturbation', zeros(size(setup.refs.q)), ...
    'feedforwardPolicy', "nominal");

if controller.id ~= "fixed_lqi" && controller.id ~= "strict_qp"
    references = evalin('base', 'references');
    references.qAbs = timeseries(nominal.q.', nominal.time(:));
    assignin('base', 'references', references);
    runtime = struct();
    runtime.model = setup.model;
    runtime.scene = controller.filterScene;
    runtime.strictConfig = strictConfig;
    runtime.schedule = controller.schedule;
    runtime.dobConfig = controller.dobConfig;
    runtime.useDob = controller.useDob;
    runtime.useStrictQp = controller.useStrictQp;
    runtime.sampleTime = config.sampleTime;
    runtime.q0 = setup.refs.q0(:);
    runtime.initialQd = nominal.qd(:, 1);
    runtime.initialForce = nominal.computedForce(:, 1);
    runtime.feedbackForceLimit = setup.config.lqiFeedbackForceLimit;
    runtime.rpyRateRcondMin = 1e-9;
    runtime.diagnosticLayout = makeCandidateControllerDiagnosticLayout();
    assignin('base', 'candidateControllerRuntime', runtime);
    setup.candidateRuntime = runtime;
    setup.candidateInterface = installSimscapeCandidateControllerRuntime(setup.modelName);
end
set_param(setup.modelName, 'StopTime', num2str(nominal.time(end), 16));
end

function setup = applyRuntimeInputs(setup, reference, experimentCase, config)
setup.refs.t = reference.time;
setup.refs.q = reference.q;
setup.refs.qd = reference.qd;
setup.refs.qdd = reference.qdd;
setup.refs.Fcomputed = reference.computedForce;
setup.refs.L = reference.legLength;
setup.refs.Ld = reference.legSpeed;
references = evalin('base', 'references');
time = reference.time(:);
references.r = timeseries((reference.q-setup.refs.q0(:)).', time);
references.qAbs = timeseries(reference.q.', time);
references.rd = timeseries(reference.qd.', time);
references.rdd = timeseries(reference.qdd.', time);
references.uCT = timeseries(reference.computedForce.', time);
references.uFF = references.uCT;
references.rL = timeseries((reference.legLength-reference.legLength(:, 1)).', time);
references.rLd = timeseries(reference.legSpeed.', time);
assignin('base', 'references', references);
setup.references = references;

wrench = evaluatePlatformWrench(reference.time, experimentCase, config);
disturbances = struct();
disturbances.Fd = timeseries(wrench.', time);
disturbances.Dw = timeseries(zeros(numel(time), 6), time);
assignin('base', 'disturbances', disturbances);
setup.disturbances = disturbances;
end

function run = failedRun(controller, experimentCase, exception)
metrics = emptyFailureMetrics();
run = struct();
run.controller = rmfield(controller, {'schedule', 'dobConfig', 'filterScene'});
run.experimentCase = experimentCase;
run.referencePolicy = "failed_before_evaluation";
run.control = struct();
run.diagnostics = struct();
run.dense = struct();
run.termination = struct('terminatedEarly', true, 'time', NaN, ...
    'reason', string(exception.identifier), 'completedControlSamples', 0, ...
    'requestedControlSamples', NaN);
run.metrics = metrics;
end

function metrics = emptyFailureMetrics()
metrics = struct('equivalentPoseRms', Inf, 'equivalentPosePeak', Inf, ...
    'recoveryTime', Inf, 'translationRms', inf(1, 3), ...
    'translationPeak', inf(1, 3), 'rotationRms', inf(1, 3), ...
    'rotationPeak', inf(1, 3), 'minLegLength', NaN, 'maxLegLength', NaN, ...
    'maxLegSpeed', Inf, 'maxLegAcceleration', Inf, 'peakForce', Inf, ...
    'peakForceRate', Inf, 'controlEnergy', Inf, 'controllerTimeP95', Inf, ...
    'qpTimeP95', Inf, 'dobTimeP95', Inf, 'infeasibleCount', 1, ...
    'fallbackCount', 1, 'minSigma', -Inf, 'minCollisionMargin', -Inf, ...
    'minCbfResidual', -Inf, 'lengthViolationCount', 1, ...
    'speedViolationCount', 1, 'accelerationViolationCount', 1, ...
    'forceViolationCount', 1, 'collisionViolationCount', 1, ...
    'forceRateViolationCount', 1, ...
    'singularityViolationCount', 1, 'nonfiniteCount', 1, ...
    'fullTrajectoryCompleted', false, 'hardConstraintsPassed', false, ...
    'eligible', false);
end

function closeWithoutSaving(modelName)
if bdIsLoaded(modelName)
    set_param(modelName, 'Dirty', 'off');
    close_system(modelName, 0);
end
end
