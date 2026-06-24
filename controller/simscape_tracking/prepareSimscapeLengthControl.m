function setup = prepareSimscapeLengthControl(trajectoryFile, bandwidthHz, useForceFeedforward)
% prepareSimscapeLengthControl - 统一准备 Simscape 参考轨迹跟踪仿真
%
% 本函数只通过工作区结构体和运行期模型参数配置 Simscape，不保存或修改 SLX。

arguments
    trajectoryFile {mustBeTextScalar} = ""
    bandwidthHz (1,1) double {mustBePositive} = 10
    useForceFeedforward (1,1) logical = true
end

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
optRoot = fullfile(projectRoot, 'opt_minimal');
controllerRoot = fullfile(projectRoot, 'controller');
if strlength(string(trajectoryFile)) == 0
    trajectoryFile = fullfile(optRoot, 'examples', ...
        'ihsid_40x20_limited_memory', 'simscape_references.mat');
end
trajectoryFile = char(trajectoryFile);
if ~isfile(trajectoryFile)
    projectRelativeFile = fullfile(projectRoot, trajectoryFile);
    if isfile(projectRelativeFile)
        trajectoryFile = projectRelativeFile;
    else
        error('prepareSimscapeLengthControl:TrajectoryFileNotFound', ...
            '未找到 Simscape 参考轨迹文件：%s', trajectoryFile);
    end
end

addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'matlab', 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
ensureSimscapeConfiguration(projectRoot);

fileVariables = string({whos('-file', trajectoryFile).name});
if ~any(fileVariables == "refs")
    error('prepareSimscapeLengthControl:InvalidReferences', ...
        '指定轨迹文件不包含 refs。');
end
sample = load(trajectoryFile, 'refs');
if any(fileVariables == "publishPassed")
    publishState = load(trajectoryFile, 'publishPassed');
else
    publishState = struct();
end
if isfield(publishState, 'publishPassed') && ~publishState.publishPassed
    error('prepareSimscapeLengthControl:TrajectoryNotPublished', ...
        '指定轨迹未通过 run_01 的求解器与工程后验双重验收。');
end
refs = validateReferences(sample.refs);

model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
simscapeData = buildSimscapeLengthControlData(model, scene);
[~, references] = exportTrajectoryToSimscape(trajectoryFromRefs(refs), scene, []);
references.rLd = timeseries(zeros(numel(refs.t), 6), refs.t(:));
references.uCT = timeseries(zeros(numel(refs.t), 6), refs.t(:));
if ~useForceFeedforward
    references.uFF = timeseries(zeros(size(references.uFF.Data)), references.uFF.Time);
end

assignin('base', 'stewart', simscapeData.stewart);
assignin('base', 'payload', simscapeData.payload);
assignin('base', 'ground', simscapeData.ground);
assignin('base', 'disturbances', simscapeData.disturbances);
assignin('base', 'references', references);
assignin('base', 'controller', initializeController('type', 'open-loop'));
assignin('base', 'Kl', ss(zeros(6)));

modelName = 'stewart_platform_model';
modelFile = fullfile(projectRoot, 'matlab', [modelName, '.slx']);
load_system(modelFile);
configureSimscapeGravity(modelName, simscapeData.gravity, 'enabled', false);
design = designSimscapeLengthController(modelName, bandwidthHz);
assignin('base', 'Kl', design.Kl);
assignin('base', 'controller', simscapeData.controller);
configureSimscapeGravity(modelName, simscapeData.gravity);
set_param(modelName, 'StopTime', num2str(refs.t(end), 16));

setup = struct();
setup.projectRoot = projectRoot;
setup.optRoot = optRoot;
setup.modelName = modelName;
setup.modelFile = modelFile;
setup.trajectoryFile = trajectoryFile;
setup.bandwidthHz = bandwidthHz;
setup.useForceFeedforward = useForceFeedforward;
setup.refs = refs;
setup.references = references;
setup.model = model;
setup.scene = scene;
setup.simscapeData = simscapeData;
setup.design = design;
end

function refs = validateReferences(refs)
% validateReferences - 校验 run_01 发布的参考轨迹字段与尺寸
requiredFields = {'t', 'q', 'qd', 'Fleg', 'L', 'q0'};
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(refs, requiredFields{fieldIndex})
        error('prepareSimscapeLengthControl:InvalidReferences', ...
            'refs 缺少必需字段 %s。', requiredFields{fieldIndex});
    end
end

time = refs.t(:);
nodeCount = numel(time);
if nodeCount < 2 || any(~isfinite(time)) || any(diff(time) <= 0)
    error('prepareSimscapeLengthControl:InvalidReferences', ...
        'refs.t 必须为严格递增的有限时间序列，且至少包含两个节点。');
end
matrixFields = {'q', 'qd', 'Fleg', 'L'};
for fieldIndex = 1:numel(matrixFields)
    value = refs.(matrixFields{fieldIndex});
    if ~isequal(size(value), [6, nodeCount]) || any(~isfinite(value), 'all')
        error('prepareSimscapeLengthControl:InvalidReferences', ...
            'refs.%s 必须为有限的 6x%d 数组。', matrixFields{fieldIndex}, nodeCount);
    end
end
if ~isequal(size(refs.q0), [6, 1]) || any(~isfinite(refs.q0))
    error('prepareSimscapeLengthControl:InvalidReferences', ...
        'refs.q0 必须为有限的 6x1 向量。');
end
refs.t = time.';
end

function traj = trajectoryFromRefs(refs)
% trajectoryFromRefs - 将已发布参考恢复为导出接口输入
traj = struct('t', refs.t, 'Q', refs.q, 'V', refs.qd, ...
    'Unode', refs.Fleg, 'L', refs.L);
end

function ensureSimscapeConfiguration(projectRoot)
% ensureSimscapeConfiguration - 确保原作者共享仿真配置已加载到基础工作区
if evalin('base', 'exist(''conf_simscape'', ''var'') == 0')
    configFile = fullfile(projectRoot, 'mat', 'conf_simscape.mat');
    evalin('base', sprintf('load(''%s'');', strrep(configFile, '''', '''''')));
end
end
