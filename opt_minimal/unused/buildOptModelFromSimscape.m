function model = buildOptModelFromSimscape(stewart)
% buildOptModelFromSimscape - 从 Simscape 工程结构整理轨迹优化模型
%
% 文件用途：
%   将现有 stewart 结构体中的几何参数转换为最小轨迹优化需要的
%   model 结构体，并补齐第一版测试用的腿长、腿速和奇异性约束。
%
% 输入参数：
%   stewart struct，可选 - 当前工程中的 Stewart 平台参数结构体。
%   若未输入，本函数会优先读取 mat/stewart.mat；若文件不可用，则
%   使用工程初始化函数生成一套默认几何。
%
% 输出参数：
%   model.A            [3x6] - 定平台下铰点 A_i，在 {A} 中表达。
%   model.B            [3x6] - 动平台上铰点 B_i，在 {B} 中表达。
%   model.L0           [6x1] - 初始腿长。
%   model.Lc           [1x1] - 归一化雅可比用的特征长度。
%   model.lmin/lmax    [6x1] - 支链长度上下限。
%   model.ldotMax      [6x1] - 支链速度上限。
%   model.sigmaMinSafe [1x1] - 最小奇异值安全阈值。
%
% 主要公式：
%   L0_i = ||B_i - A_i||，因为 q=0 时 {B} 与 {A} 重合。
%   lmin = L0 - strokeDefault/2，lmax = L0 + strokeDefault/2。
%
% 与 Stewart 平台轨迹优化的关系：
%   该函数是 Simscape 参数和 MATLAB 优化器之间的隔离层。后续若
%   Simscape 模型中加入真实电动缸行程或速度字段，只需在这里适配。

if nargin < 1 || isempty(stewart)
    stewart = loadOrCreateStewart();
end

[A, B] = extractJointGeometry(stewart);
validateattributes(A, {'double'}, {'real', 'finite', 'size', [3, 6]}, mfilename, 'model.A');
validateattributes(B, {'double'}, {'real', 'finite', 'size', [3, 6]}, mfilename, 'model.B');

if isfield(stewart, 'geometry') && isfield(stewart.geometry, 'l')
    L0 = stewart.geometry.l(:);
else
    L0 = vecnorm(B - A, 2, 1).';
end
validateattributes(L0, {'double'}, {'real', 'finite', 'size', [6, 1], 'positive'}, mfilename, 'model.L0');

strokeDefault = 0.04;      % m，第一版测试默认行程，后续应替换为真实电动缸参数。
ldotDefault = 0.02;        % m/s，第一版测试默认速度上限。
sigmaMinSafeDefault = 0.05; % 归一化雅可比最小奇异值安全阈值。

model = struct();
model.A = A;
model.B = B;
model.L0 = L0;
model.strokeDefault = strokeDefault;
model.lmin = L0 - strokeDefault/2;
model.lmax = L0 + strokeDefault/2;
model.ldotMax = ldotDefault * ones(6, 1);
model.sigmaMinSafe = sigmaMinSafeDefault;
model.Lc = estimateCharacteristicLength(A, B, L0);
model.rpyOrder = 'ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll)';
model.source = 'stewart-simscape geometry';
model.weights = struct('wa', 1.0, 'wl', 0.1);

% 若用户已在 stewart.actuators 中放入同名字段，则优先采用真实参数。
if isfield(stewart, 'actuators')
    model = copyVectorFieldIfPresent(model, stewart.actuators, 'lmin', 6);
    model = copyVectorFieldIfPresent(model, stewart.actuators, 'lmax', 6);
    model = copyVectorFieldIfPresent(model, stewart.actuators, 'ldotMax', 6);
end

if any(model.lmin >= model.lmax)
    error('buildOptModelFromSimscape:InvalidLengthBounds', ...
        '支链长度上下限不合法：需要逐项满足 lmin < lmax。');
end
end

function stewart = loadOrCreateStewart()
projectRoot = fileparts(fileparts(mfilename('fullpath')));
matFile = fullfile(projectRoot, 'mat', 'stewart.mat');

if isfile(matFile)
    loadedData = load(matFile);
    if isfield(loadedData, 'stewart')
        stewart = loadedData.stewart;
        return;
    end
end

srcPath = fullfile(projectRoot, 'src');
if exist(srcPath, 'dir')
    addpath(srcPath);
end

requiredFunctions = {'initializeStewartPlatform', 'initializeFramesPositions', ...
    'generateGeneralConfiguration', 'computeJointsPose'};
for functionIndex = 1:numel(requiredFunctions)
    if exist(requiredFunctions{functionIndex}, 'file') ~= 2
        error('buildOptModelFromSimscape:MissingFunction', ...
            '无法找到工程初始化函数 %s，请确认已在 stewart-simscape 根目录运行。', ...
            requiredFunctions{functionIndex});
    end
end

stewart = initializeStewartPlatform();
stewart = initializeFramesPositions(stewart, 'H', 90e-3, 'MO_B', 45e-3);
stewart = generateGeneralConfiguration(stewart);
stewart = computeJointsPose(stewart);
end

function [A, B] = extractJointGeometry(stewart)
if isfield(stewart, 'geometry') && isfield(stewart.geometry, 'Aa') && isfield(stewart.geometry, 'Bb')
    A = stewart.geometry.Aa;
    B = stewart.geometry.Bb;
    return;
end

if isfield(stewart, 'Aa') && isfield(stewart, 'Bb')
    A = orientJointArray(stewart.Aa, 'stewart.Aa');
    B = orientJointArray(stewart.Bb, 'stewart.Bb');
    [A, B] = convertMillimeterGeometryIfNeeded(A, B);
    return;
end

if isfield(stewart, 'platform_F') && isfield(stewart.platform_F, 'Fa') && ...
        isfield(stewart.platform_F, 'FO_A') && ...
        isfield(stewart, 'platform_M') && isfield(stewart.platform_M, 'Mb') && ...
        isfield(stewart.platform_M, 'MO_B')
    A = stewart.platform_F.Fa - stewart.platform_F.FO_A;
    B = stewart.platform_M.Mb - stewart.platform_M.MO_B;
    return;
end

error('buildOptModelFromSimscape:MissingGeometry', ...
    ['无法从 stewart 中读取铰点几何。需要 stewart.geometry.Aa/Bb，' ...
     '或 platform_F.Fa/FO_A 与 platform_M.Mb/MO_B。']);
end

function joints = orientJointArray(rawJoints, fieldName)
validateattributes(rawJoints, {'double'}, {'real', 'finite', '2d'}, mfilename, fieldName);
if isequal(size(rawJoints), [3, 6])
    joints = rawJoints;
elseif isequal(size(rawJoints), [6, 3])
    joints = rawJoints.';
else
    error('buildOptModelFromSimscape:InvalidJointSize', ...
        '%s 的尺寸必须是 3x6 或 6x3。', fieldName);
end
end

function [A, B] = convertMillimeterGeometryIfNeeded(A, B)
if max(abs([A(:); B(:)])) > 2
    % mat/stewart.mat 中的旧结构使用 mm；优化模型统一使用 SI 单位 m。
    A = A / 1000;
    B = B / 1000;
end
end

function Lc = estimateCharacteristicLength(A, B, L0)
fixedRadius = mean(vecnorm(A(1:2, :), 2, 1));
mobileRadius = mean(vecnorm(B(1:2, :), 2, 1));
radiusMean = mean([fixedRadius, mobileRadius]);

if isfinite(radiusMean) && radiusMean > 1e-6
    Lc = radiusMean;
else
    Lc = 0.5 * mean(L0);
end

if ~isfinite(Lc) || Lc <= 0
    error('buildOptModelFromSimscape:InvalidLc', '无法估计正的特征长度 Lc。');
end
end

function model = copyVectorFieldIfPresent(model, sourceStruct, fieldName, expectedLength)
if ~isfield(sourceStruct, fieldName)
    return;
end

fieldValue = sourceStruct.(fieldName);
if isscalar(fieldValue)
    fieldValue = fieldValue * ones(expectedLength, 1);
else
    fieldValue = fieldValue(:);
end

if numel(fieldValue) ~= expectedLength || any(~isfinite(fieldValue))
    error('buildOptModelFromSimscape:InvalidField', ...
        'stewart.actuators.%s 必须是有限标量或 %dx1 向量。', fieldName, expectedLength);
end

model.(fieldName) = fieldValue;
end
