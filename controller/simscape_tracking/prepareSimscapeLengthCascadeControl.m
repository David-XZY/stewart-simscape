function setup = prepareSimscapeLengthCascadeControl(trajectoryFile, configOverrides)
% prepareSimscapeLengthCascadeControl - 准备纯 q/qd 长度串级控制仿真
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

fileVariables = string({whos('-file', trajectoryFile).name});
if ~any(fileVariables == "refs")
    error('prepareSimscapeLengthCascadeControl:InvalidReferences', ...
        '指定轨迹文件不包含 refs。');
end
sample = load(trajectoryFile, 'refs');

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
config = makeSimscapeLengthCascadeConfig(model, configOverrides);
[lengthRefs, references] = generateSimscapeLengthCascadeReferences( ...
    sample.refs, model, config.sampleTime);
design = designSimscapeLengthCascadeController(config);
simscapeData = buildSimscapeLengthControlData(model, scene, 'length-cascade');
controller = simscapeData.controller;

assignin('base', 'stewart', simscapeData.stewart);
assignin('base', 'payload', simscapeData.payload);
assignin('base', 'ground', simscapeData.ground);
assignin('base', 'disturbances', simscapeData.disturbances);
assignin('base', 'references', references);
assignin('base', 'controller', controller);
assignin('base', 'lengthCascadeConfig', config);
assignin('base', 'lengthCascadeDesign', design);
assignin('base', 'lengthServoConfig', config);

modelName = 'stewart_platform_model';
modelFile = fullfile(projectRoot, 'matlab', [modelName, '.slx']);
load_system(modelFile);
configureSimscapeGravity(modelName, simscapeData.gravity, ...
    'enabled', config.gravityEnabled);
set_param(modelName, 'StopTime', num2str(lengthRefs.t(end), 16));

setup = struct();
setup.projectRoot = projectRoot;
setup.optRoot = optRoot;
setup.modelName = modelName;
setup.modelFile = modelFile;
setup.trajectoryFile = trajectoryFile;
setup.refs = lengthRefs;
setup.references = references;
setup.model = model;
setup.scene = scene;
setup.simscapeData = simscapeData;
setup.controller = controller;
setup.config = config;
setup.design = design;
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
error('prepareSimscapeLengthCascadeControl:TrajectoryFileNotFound', ...
    '未找到长度串级参考轨迹文件：%s', trajectoryFile);
end

function ensureSimscapeConfiguration(projectRoot)
if evalin('base', 'exist(''conf_simscape'', ''var'') == 0')
    configFile = fullfile(projectRoot, 'mat', 'conf_simscape.mat');
    evalin('base', sprintf('load(''%s'');', strrep(configFile, '''', '''''')));
end
end
