function config = makeDisturbanceComparisonConfig(overrides)
% makeDisturbanceComparisonConfig - Canonical disturbance-comparison contract.
arguments
    overrides struct = struct()
end

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
config = struct();
config.controllerIds = ["fixed_lqi", "strict_qp", "scheduled_lqi", ...
    "fixed_lqi_dob", "scheduled_lqi_dob", ...
    "scheduled_lqi_dob_strict_qp"];
config.baselineControllerIds = ["fixed_lqi", "strict_qp"];
config.candidateControllerIds = ["scheduled_lqi", "fixed_lqi_dob", ...
    "scheduled_lqi_dob", "scheduled_lqi_dob_strict_qp"];
config.disturbanceScales = [0.5, 1.0, 1.5];
config.basePlatformWrench = [100; -80; 60; 6; -5; 4];
config.baseAttitudeBumpDeg = [0.5; -0.4; 0.3];
config.baseAttitudeNoiseStdDeg = [0.05; 0.05; 0.08];
config.noiseSeeds = 101:105;
config.sampleTime = 0.01;
config.trajectoryFile = fullfile(projectRoot, 'opt_minimal', 'examples', ...
    'ihsid_40x20_limited_memory', 'simscape_references.mat');
config.outputDir = fullfile(projectRoot, 'results', 'reports', ...
    'disturbance_control_comparison');
config.evidenceDir = fullfile(projectRoot, 'docs', 'experiments', ...
    'disturbance-control-comparison');
config.wrenchWindow = [2.50, 2.90];
config.bumpWindow = [4.50, 5.30];
config.bumpRiseTime = 0.20;
config.noiseWindow = [1.0, 6.5];
config.noiseCutoffHz = 2.0;
config.dobCutoffHzGrid = [2, 5, 8];
config.dobForceLimitGrid = [60, 120, 180];
config.dobRateLimitGrid = [1000, 2000, 4000];
config.commandGovernorCutoffHz = 2.0;
config.commandGovernorDampingRatio = 1.0;
config.commandGovernorMaxRateDegPerSec = [2; 2; 3];
config.commandGovernorMaxAccelerationDegPerSec2 = [20; 20; 30];
config.qpAwareAntiWindupMismatchLimit = 30;
config.qpAwareAntiWindupGain = 0.1;
config.trainingScale = 1.0;
config.trainingNoiseSeeds = 101:103;
config.holdoutNoiseSeeds = 104:105;
config.integrationSubsteps = 2;
config.screeningSkipCollision = true;
config.useFastRestart = false;
config.initialPoseOffset = [0.0004; -0.0003; 0.0002; ...
    0.0002; -0.00015; 0.0001];
config.initialVelocityOffset = zeros(6, 1);
config.stateStep = [1e-6*ones(6, 1); 1e-5*ones(6, 1)];
config.inputStep = 1;
config.hardConstraintTolerance = 1e-7;
config.stateConstraintTolerance = 2e-6;
config.recoveryBand = 1.10;
config.recoveryHoldTime = 0.20;
config.minimumImprovement = 0.15;
config.maximumCombinedRegression = 0.10;
config.onlineP95Limit = 0.010;
config.strictOverrides = struct();
config.baselineOverrides = struct();
config.writeArtifacts = true;
config.exportFigures = true;
config.runExactScreening = true;
config.runFullExactMatrix = true;
config.runSimscapeFinalists = true;
config.caseIds = strings(0, 1);
config.simscapeCaseIds = strings(0, 1);
config.failOnRejected = false;

config = mergeKnown(config, overrides);
validateConfig(config);
end

function target = mergeKnown(target, source)
names = fieldnames(source);
for index = 1:numel(names)
    name = names{index};
    if ~isfield(target, name)
        error('makeDisturbanceComparisonConfig:UnknownOption', ...
            'Unknown comparison option: %s.', name);
    end
    target.(name) = source.(name);
end
end

function validateConfig(config)
validControllers = ["fixed_lqi", "strict_qp", "scheduled_lqi", ...
    "fixed_lqi_dob", "scheduled_lqi_dob", ...
    "scheduled_lqi_dob_strict_qp"];
if isempty(config.controllerIds) || any(~ismember(string(config.controllerIds), validControllers))
    error('makeDisturbanceComparisonConfig:InvalidController', ...
        'controllerIds contains an unsupported controller identifier.');
end
validateattributes(config.disturbanceScales, {'double'}, ...
    {'vector', 'positive', 'finite', 'increasing'});
validateattributes(config.basePlatformWrench, {'double'}, ...
    {'real', 'finite', 'size', [6, 1]});
validateattributes(config.baseAttitudeBumpDeg, {'double'}, ...
    {'real', 'finite', 'size', [3, 1]});
validateattributes(config.baseAttitudeNoiseStdDeg, {'double'}, ...
    {'real', 'finite', 'positive', 'size', [3, 1]});
validateattributes(config.noiseSeeds, {'double'}, ...
    {'vector', 'integer', 'finite', 'nonnegative'});
validateattributes(config.sampleTime, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.commandGovernorCutoffHz, {'double'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(config.commandGovernorDampingRatio, {'double'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(config.qpAwareAntiWindupMismatchLimit, {'double'}, ...
    {'scalar', 'positive', 'finite'});
validateattributes(config.qpAwareAntiWindupGain, {'double'}, ...
    {'scalar', 'positive', 'finite'});
makeAttitudeCommandGovernorConfig(config.sampleTime, ...
    config.commandGovernorCutoffHz, config.commandGovernorDampingRatio, ...
    config.commandGovernorMaxRateDegPerSec, ...
    config.commandGovernorMaxAccelerationDegPerSec2);
if config.bumpRiseTime <= 0 || 2*config.bumpRiseTime > diff(config.bumpWindow)
    error('makeDisturbanceComparisonConfig:InvalidBumpWindow', ...
        'bumpRiseTime must fit twice inside bumpWindow.');
end
if config.noiseCutoffHz >= 0.5/config.sampleTime
    error('makeDisturbanceComparisonConfig:InvalidNoiseCutoff', ...
        'noiseCutoffHz must be below Nyquist.');
end
end
