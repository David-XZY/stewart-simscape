function setup = prepareSimscapePoseForceControl(trajectoryFile, configOverrides)
% prepareSimscapePoseForceControl - 准备重力开启的力输入位姿轨迹跟踪仿真
arguments
    trajectoryFile {mustBeTextScalar} = ""
    configOverrides struct = struct()
end

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
optRoot = fullfile(projectRoot, 'opt_minimal');
if strlength(string(trajectoryFile)) == 0
    trajectoryFile = fullfile(optRoot, 'examples', ...
        'ihsid_40x20_limited_memory', 'simscape_references.mat');
end
trajectoryFile = resolveTrajectoryFile(char(trajectoryFile), projectRoot);

addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));
ensureSimscapeConfiguration(projectRoot);

fileVariables = string({whos('-file', trajectoryFile).name});
if ~any(fileVariables == "refs")
    error('prepareSimscapePoseForceControl:InvalidReferences', ...
        '指定轨迹文件不包含 refs。');
end
sample = load(trajectoryFile, 'refs');
if any(fileVariables == "publishPassed")
    publishState = load(trajectoryFile, 'publishPassed');
    if ~publishState.publishPassed
        error('prepareSimscapePoseForceControl:TrajectoryNotPublished', ...
            '指定轨迹未通过 run_01 的求解器与工程后验双重验收。');
    end
end
nodeRefs = validateReferences(sample.refs);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
config = makeSimscapePoseForceConfig(model, configOverrides);
refs = reconstructPoseForceReferences(nodeRefs, model, config.derivativeSampleTime);
simscapeData = buildSimscapeLengthControlData(model, scene);
[~, references] = exportTrajectoryToSimscape(trajectoryFromRefs(refs), scene, []);
references.rLd = timeseries(refs.Ld.', refs.t(:));
references.description = '由节点 q/qd 经三次 Hermite 重建的力输入位姿跟踪参考轨迹';

assignin('base', 'stewart', simscapeData.stewart);
assignin('base', 'payload', simscapeData.payload);
assignin('base', 'ground', simscapeData.ground);
assignin('base', 'disturbances', simscapeData.disturbances);
assignin('base', 'references', references);
assignin('base', 'controller', initializeController('type', 'open-loop'));
assignin('base', 'K', ss(zeros(6)));

modelName = 'stewart_platform_model';
modelFile = fullfile(projectRoot, 'matlab', [modelName, '.slx']);
load_system(modelFile);
configureSimscapeGravity(modelName, simscapeData.gravity, 'enabled', false);
referenceCleanup = zeroForceFeedforward(); %#ok<NASGU>
design = designSimscapePoseForceController(modelName, model, config);
clear referenceCleanup;

controller = initializeController('type', 'ref-track-X');
assignin('base', 'K', design.K);
assignin('base', 'controller', controller);
configureSimscapeGravity(modelName, simscapeData.gravity, 'enabled', config.gravityEnabled);
set_param(modelName, 'StopTime', num2str(refs.t(end), 16));

setup = struct();
setup.projectRoot = projectRoot;
setup.optRoot = optRoot;
setup.modelName = modelName;
setup.modelFile = modelFile;
setup.trajectoryFile = trajectoryFile;
setup.refs = refs;
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
error('prepareSimscapePoseForceControl:TrajectoryFileNotFound', ...
    '未找到 Simscape 参考轨迹文件：%s', trajectoryFile);
end

function refs = validateReferences(refs)
requiredFields = {'t', 'q', 'qd', 'Fleg', 'L', 'q0'};
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(refs, requiredFields{fieldIndex})
        error('prepareSimscapePoseForceControl:InvalidReferences', ...
            'refs 缺少必需字段 %s。', requiredFields{fieldIndex});
    end
end
time = refs.t(:);
nodeCount = numel(time);
if nodeCount < 2 || any(~isfinite(time)) || any(diff(time) <= 0)
    error('prepareSimscapePoseForceControl:InvalidReferences', ...
        'refs.t 必须为严格递增的有限时间序列。');
end
for fieldName = {'q', 'qd', 'Fleg', 'L'}
    value = refs.(fieldName{1});
    if ~isequal(size(value), [6, nodeCount]) || any(~isfinite(value), 'all')
        error('prepareSimscapePoseForceControl:InvalidReferences', ...
            'refs.%s 必须为有限的 6x%d 数组。', fieldName{1}, nodeCount);
    end
end
refs.t = time.';
end

function traj = trajectoryFromRefs(refs)
traj = struct('t', refs.t, 'Q', refs.q, 'V', refs.qd, ...
    'Unode', refs.Fleg, 'L', refs.L);
end

function refs = reconstructPoseForceReferences(nodeRefs, model, sampleTime)
% reconstructPoseForceReferences - 重建位姿参考并在同一网格计算腿长、腿速与前馈力
poseReference = reconstructHermitePoseReference( ...
    nodeRefs.t, nodeRefs.q, nodeRefs.qd, sampleTime);
refs = nodeRefs;
refs.t = poseReference.t;
refs.q = poseReference.q;
refs.qd = poseReference.qd;
refs.nodeTime = poseReference.nodeTime;
refs.referenceInterpolation = poseReference.interpolation;
refs.Fleg = interp1(nodeRefs.t(:), nodeRefs.Fleg.', refs.t(:), 'linear').';

sampleCount = numel(refs.t);
refs.L = zeros(6, sampleCount);
refs.Ld = zeros(6, sampleCount);
for sampleIndex = 1:sampleCount
    kin = sgpIK(refs.q(:, sampleIndex), model);
    jacobian = sgpJacobian(refs.q(:, sampleIndex), model);
    refs.L(:, sampleIndex) = kin.L;
    refs.Ld(:, sampleIndex) = jacobian.Jq * refs.qd(:, sampleIndex);
end
end

function ensureSimscapeConfiguration(projectRoot)
if evalin('base', 'exist(''conf_simscape'', ''var'') == 0')
    configFile = fullfile(projectRoot, 'mat', 'conf_simscape.mat');
    evalin('base', sprintf('load(''%s'');', strrep(configFile, '''', '''''')));
end
end

function cleanup = zeroForceFeedforward()
references = evalin('base', 'references');
originalReferences = references;
references.uFF = timeseries(zeros(size(references.uFF.Data)), references.uFF.Time);
assignin('base', 'references', references);
cleanup = onCleanup(@() assignin('base', 'references', originalReferences));
end
