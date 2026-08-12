function setup = prepareSimscapeStrictClfCbfQp(trajectoryFile, options)
% prepareSimscapeStrictClfCbfQp - Prepare an in-memory strict-QP Simscape run.
arguments
    trajectoryFile {mustBeTextScalar} = ""
    options struct = struct()
end

settings = defaultSettings();
settings = mergeKnownFields(settings, options, ...
    'prepareSimscapeStrictClfCbfQp:UnknownOption');
settings.actuatorMode = string(validatestring(settings.actuatorMode, ...
    {'ideal-force', 'nonideal-force'}));
validateattributes(settings.sampleTime, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(settings.rpyRateRcondMin, {'double'}, ...
    {'scalar', 'positive', 'finite', '<', 1});
validateattributes(settings.collisionSampleHoldMargin, {'double'}, ...
    {'scalar', 'nonnegative', 'finite'});

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
controllerRoot = fullfile(projectRoot, 'controller');
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'strict_clf_cbf_qp'));

baselineOverrides = settings.baselineOverrides;
baselineOverrides.controlLaw = 'computed-torque';
baselineOverrides.actuatorMode = char(settings.actuatorMode);
baselineOverrides.derivativeSampleTime = settings.sampleTime;
baselineSetup = prepareSimscapePoseForceControl(trajectoryFile, baselineOverrides);
[baselineSetup, plantScene, perturbation] = applyPlantPerturbation( ...
    baselineSetup, settings.plantPerturbation);
if max(abs(baselineSetup.refs.q0(4:6))) > 1e-12
    error('prepareSimscapeStrictClfCbfQp:NonzeroHomeOrientation', ...
        ['The current direct-pose bridge reconstructs q=q0+Xr and therefore ' ...
         'requires zero home RPY.']);
end

strictScene = baselineSetup.scene;
strictScene.collision.safeDistance = baselineSetup.scene.collision.safeDistance+ ...
    settings.collisionSampleHoldMargin;
strictScene.collision.stage1ConstraintDistance = ...
    baselineSetup.scene.collision.stage1ConstraintDistance+ ...
    settings.collisionSampleHoldMargin;
strictScene.collision.finalGap = baselineSetup.scene.collision.finalGap+ ...
    settings.collisionSampleHoldMargin;
strictOverrides = settings.strictOverrides;
strictOverrides = applySimscapeStrictDefaults(strictOverrides, ...
    baselineSetup.scene, settings.collisionSampleHoldMargin);
strictOverrides.dt = settings.sampleTime;
strictConfig = makeStrictClfCbfQpConfig(baselineSetup.model, strictOverrides);
references = evalin('base', 'references');
references.qAbs = timeseries(baselineSetup.refs.q.', baselineSetup.refs.t(:));
assignin('base', 'references', references);

initialForce = baselineSetup.refs.Fcomputed(:, 1);
initialForce = min(max(initialForce, strictConfig.forceMin), strictConfig.forceMax);
% Build CasADi maps and exercise quadprog before simulation.  Cold-start
% cost is retained separately and must not be presented as online solve time.
coldTimer = tic;
warmupSolveTimes = nan(20, 1);
warmupStepTimes = nan(20, 1);
initialDiagnostic = struct();
warmupCount = 0;
for warmupIndex = 1:numel(warmupSolveTimes)
    stepTimer = tic;
    [~, initialDiagnostic] = stepStrictClfCbfQp( ...
        baselineSetup.refs.q(:, 1), baselineSetup.refs.qd(:, 1), ...
        baselineSetup.refs.q(:, 1), baselineSetup.refs.qd(:, 1), ...
        baselineSetup.refs.qdd(:, 1), initialForce, initialForce, ...
        baselineSetup.model, strictScene, strictConfig);
    warmupStepTimes(warmupIndex) = toc(stepTimer);
    warmupSolveTimes(warmupIndex) = initialDiagnostic.solveTime;
    warmupCount = warmupIndex;
    if warmupIndex >= 3 && all(warmupStepTimes(warmupIndex-2:warmupIndex) <= ...
            strictConfig.dt)
        break;
    end
end
coldInitializationWallTime = toc(coldTimer);
warmupSolveTimes = warmupSolveTimes(1:warmupCount);
warmupStepTimes = warmupStepTimes(1:warmupCount);
initialAudit = struct( ...
    'minimumStateMargin', min(initialDiagnostic.cbf.stateMargins), ...
    'minimumPsi1', min(initialDiagnostic.cbf.psi1), ...
    'minimumHardResidual', min(initialDiagnostic.cbfResidual), ...
    'qpFeasible', initialDiagnostic.feasible, ...
    'usedFallback', initialDiagnostic.usedFallback, ...
    'coldInitializationWallTime', coldInitializationWallTime, ...
    'coldQpSolveTime', warmupSolveTimes(1), ...
    'coldControllerStepTime', warmupStepTimes(1), ...
    'warmupCount', warmupCount, ...
    'warmupSolveTimes', warmupSolveTimes, ...
    'warmupStepTimes', warmupStepTimes, ...
    'warmControllerStepTime', warmupStepTimes(end));
initialTolerance = 10*strictConfig.solver.constraintTolerance;
if initialAudit.minimumStateMargin < -initialTolerance || ...
        initialAudit.minimumPsi1 < -initialTolerance
    error('prepareSimscapeStrictClfCbfQp:InvalidInitialHocbfState', ...
        ['Initial HOCBF recursion is not admissible: min h=%.6g, ' ...
         'min psi1=%.6g.'], initialAudit.minimumStateMargin, ...
        initialAudit.minimumPsi1);
end
if ~initialAudit.qpFeasible || initialAudit.usedFallback
    error('prepareSimscapeStrictClfCbfQp:InitialQpInfeasible', ...
        'The prewarmed initial strict QP is not feasible.');
end
runtime = struct();
runtime.model = baselineSetup.model;
runtime.scene = strictScene;
runtime.physicalScene = baselineSetup.scene;
runtime.config = strictConfig;
runtime.q0 = baselineSetup.refs.q0(:);
runtime.initialForce = initialForce;
runtime.initialQd = baselineSetup.refs.qd(:, 1);
runtime.initialReferencePose = baselineSetup.refs.q(:, 1);
runtime.initialReferenceVelocity = baselineSetup.refs.qd(:, 1);
runtime.initialReferenceAcceleration = baselineSetup.refs.qdd(:, 1);
runtime.sampleTime = settings.sampleTime;
runtime.rpyRateRcondMin = settings.rpyRateRcondMin;
runtime.diagnosticLayout = makeStrictQpSimscapeDiagnosticLayout();
runtime.initialConditionAudit = initialAudit;
runtime.actuatorMode = settings.actuatorMode;
runtime.nominalForceDefinition = ...
    'F_nom = F_CT + F_LQI at the output of Sum Feedback Feedforward';
runtime.poseVelocityDefinition = ...
    'qdot=[world linear velocity; E_ZYX(q)^{-1} world angular velocity]';
assignin('base', 'strictQpSimscape', runtime);
assignin('base', 'strictClfCbfConfig', strictConfig);

interface = installSimscapeStrictClfCbfQpRuntime(baselineSetup.modelName);
controller = initializeController('type', 'strict-clf-cbf-qp');
assignin('base', 'controller', controller);
set_param(baselineSetup.modelName, 'StopTime', ...
    num2str(baselineSetup.refs.t(end), 16));

setup = baselineSetup;
setup.references = references;
setup.controller = controller;
setup.baselineConfig = baselineSetup.config;
setup.strictConfig = strictConfig;
setup.strictScene = strictScene;
setup.plantScene = plantScene;
setup.plantPerturbation = perturbation;
setup.strictRuntime = runtime;
setup.strictInterface = interface;
setup.actuatorMode = settings.actuatorMode;
setup.options = settings;
if settings.actuatorMode == "ideal-force"
    setup.validationClass = "matched-parameter ideal-force Simscape validation";
else
    setup.validationClass = "nonideal-actuator engineering robustness validation";
end
end

function overrides = applySimscapeStrictDefaults(overrides, scene, sampleHoldMargin)
% alpha=12 1/s is selected from the maximum approach speed versus the
% remaining roof clearance; it preserves positive collision psi1 on the
% published two-stage trajectory without weakening any hard boundary.
if ~isfield(overrides, 'cbf')
    overrides.cbf = struct();
end
if ~isfield(overrides.cbf, 'collisionAlpha1')
    overrides.cbf.collisionAlpha1 = 12.0;
end
if ~isfield(overrides.cbf, 'collisionAlpha2')
    overrides.cbf.collisionAlpha2 = 12.0;
end
if ~isfield(overrides, 'collision')
    overrides.collision = struct();
end
if ~isfield(overrides.collision, 'roofStage1Distance')
    overrides.collision.roofStage1Distance = ...
        scene.collision.stage1ConstraintDistance+sampleHoldMargin;
end
if ~isfield(overrides.collision, 'roofFinalDistance')
    overrides.collision.roofFinalDistance = ...
        scene.collision.finalGap+sampleHoldMargin;
end
if ~isfield(overrides.collision, 'sideDistance')
    overrides.collision.sideDistance = ...
        scene.collision.safeDistance+sampleHoldMargin;
end
end

function settings = defaultSettings()
settings = struct();
settings.actuatorMode = "ideal-force";
settings.sampleTime = 0.01;
settings.rpyRateRcondMin = 1e-9;
settings.collisionSampleHoldMargin = 5e-4;
settings.baselineOverrides = struct();
settings.strictOverrides = struct();
settings.plantPerturbation = struct();
end

function [setup, plantScene, perturbation] = applyPlantPerturbation(setup, requested)
perturbation = struct('payloadMassScale', 1.0, ...
    'payloadComOffsetP', zeros(3, 1), 'forceLagScale', 1.0);
perturbation = mergeKnownFields(perturbation, requested, ...
    'prepareSimscapeStrictClfCbfQp:UnknownPlantPerturbation');
validateattributes(perturbation.payloadMassScale, {'double'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(perturbation.payloadComOffsetP, {'double'}, ...
    {'real', 'finite', 'size', [3, 1]});
validateattributes(perturbation.forceLagScale, {'double'}, ...
    {'scalar', 'positive', 'finite'});

payload = setup.simscapeData.payload;
payload.m = payload.m*perturbation.payloadMassScale;
payload.I = payload.I*perturbation.payloadMassScale;
payload.I_local = payload.I_local*perturbation.payloadMassScale;
payload.center = payload.center+perturbation.payloadComOffsetP;
setup.simscapeData.payload = payload;
assignin('base', 'payload', payload);

plantScene = setup.scene;
plantScene.objectCylinder.center_P = plantScene.objectCylinder.center_P+ ...
    perturbation.payloadComOffsetP;
if setup.config.actuatorMode == "nonideal-force"
    actuator = setup.simscapeData.nonidealForceActuator;
    actuator.timeConstant = actuator.timeConstant*perturbation.forceLagScale;
    setup.simscapeData.nonidealForceActuator = actuator;
    setup.simscapeData.stewart.nonidealForceActuator = actuator;
    assignin('base', 'nonidealForceActuator', actuator);
    assignin('base', 'stewart', setup.simscapeData.stewart);
elseif abs(perturbation.forceLagScale-1) > eps
    error('prepareSimscapeStrictClfCbfQp:LagPerturbationRequiresNonidealActuator', ...
        'forceLagScale requires actuatorMode="nonideal-force".');
end
end

function target = mergeKnownFields(target, source, identifier)
fields = fieldnames(source);
for index = 1:numel(fields)
    name = fields{index};
    if ~isfield(target, name)
        error(identifier, 'Unknown option: %s.', name);
    end
    target.(name) = source.(name);
end
end
