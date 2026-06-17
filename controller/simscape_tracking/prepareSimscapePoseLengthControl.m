function setup = prepareSimscapePoseLengthControl(trajectoryFile, configOverrides)
% prepareSimscapePoseLengthControl - 准备位姿外环加纯腿长输入控制
arguments
    trajectoryFile {mustBeTextScalar} = ""
    configOverrides struct = struct()
end

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
optRoot = fullfile(projectRoot, 'opt_minimal');
controllerRoot = fullfile(projectRoot, 'controller');
if strlength(string(trajectoryFile)) == 0
    trajectoryFile = fullfile(optRoot, 'examples', ...
        'ihsid_40x20_limited_memory', 'simscape_references.mat');
end
trajectoryFile = resolveTrajectoryFile(char(trajectoryFile), projectRoot);

addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'matlab', 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
ensureSimscapeConfiguration(projectRoot);

sample = load(trajectoryFile, 'refs');
if ~isfield(sample, 'refs')
    error('prepareSimscapePoseLengthControl:InvalidReferences', ...
        '指定轨迹文件不包含 refs。');
end
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
config = makeSimscapePoseLengthConfig(model, configOverrides);
[refs, references] = generateSimscapeLengthCascadeReferences( ...
    sample.refs, model, config.sampleTime);
design = designSimscapePoseLengthController(model, config);
simscapeData = buildSimscapeLengthControlData(model, scene, 'pose-length-cascade');
controller = simscapeData.controller;

assignin('base', 'stewart', simscapeData.stewart);
assignin('base', 'payload', simscapeData.payload);
assignin('base', 'ground', simscapeData.ground);
assignin('base', 'disturbances', simscapeData.disturbances);
assignin('base', 'references', references);
assignin('base', 'controller', controller);
assignin('base', 'poseLengthConfig', config);
assignin('base', 'poseLengthDesign', design);
assignin('base', 'lengthServoConfig', config);

modelName = 'stewart_platform_model';
modelFile = fullfile(projectRoot, 'matlab', [modelName, '.slx']);
load_system(modelFile);
configureSimscapeGravity(modelName, simscapeData.gravity, 'enabled', config.gravityEnabled);
set_param(modelName, 'StopTime', num2str(refs.t(end), 16));

setup = struct('projectRoot', projectRoot, 'optRoot', optRoot, ...
    'modelName', modelName, 'modelFile', modelFile, ...
    'trajectoryFile', trajectoryFile, 'refs', refs, 'references', references, ...
    'model', model, 'scene', scene, 'simscapeData', simscapeData, ...
    'controller', controller, 'config', config, 'design', design);
end

function trajectoryFile = resolveTrajectoryFile(trajectoryFile, projectRoot)
if isfile(trajectoryFile)
    return;
end
projectRelativeFile = fullfile(projectRoot, trajectoryFile);
if isfile(projectRelativeFile)
    trajectoryFile = projectRelativeFile;
    return;
end
error('prepareSimscapePoseLengthControl:TrajectoryFileNotFound', ...
    '未找到位姿外环长度输入参考轨迹：%s', trajectoryFile);
end

function ensureSimscapeConfiguration(projectRoot)
if evalin('base', 'exist(''conf_simscape'', ''var'') == 0')
    configFile = fullfile(projectRoot, 'mat', 'conf_simscape.mat');
    evalin('base', sprintf('load(''%s'');', strrep(configFile, '''', '''''')));
end
end
