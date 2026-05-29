function kin = sgpIK(q, model)
% sgpIK - Stewart 平台最小坐标逆运动学
%
% 文件用途：
%   根据平台广义坐标 q 计算六条支链的当前长度、方向和上铰点位置。
%
% 输入参数：
%   q     [6x1] - [x; y; z; roll; pitch; yaw]，表示 {B} 相对 {A} 的位姿。
%   model struct - buildOptModelCustom 输出或等价结构中的几何和约束参数。
%
% 输出参数：
%   kin.L  [6x1] - 六条支链长度。
%   kin.s  [3x6] - 支链矢量，s_i = p + R*B_leg_i - A_i。
%   kin.u  [3x6] - 支链单位方向向量，u_i = s_i / ||s_i||。
%   kin.rB [3x6] - r_i = R*B_leg_i，上铰点相对动平台原点的空间向量。
%   kin.R  [3x3] - 当前姿态旋转矩阵。
%
% 主要公式：
%   R = rpy2rotmZYX([roll; pitch; yaw]);
%   s_i = p + R*B_leg_i - A_i;
%   L_i = norm(s_i);
%   u_i = s_i / L_i;
%   r_i = R*B_i。
%
% 与 Stewart 平台轨迹优化的关系：
%   腿长约束、腿速约束和雅可比奇异性约束都建立在该逆运动学结果上。
%   A_i 是定平台下铰点，B_leg_i 是第 i 条腿实际连接的动平台上铰点。
%   若 model.legMap=[6 1 2 3 4 5]，则第 1 条腿连接 A1-B6，
%   第 2 条腿连接 A2-B1，依此类推。

validateattributes(q, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'q');
q = q(:);
validateModelGeometry(model);

p = q(1:3);
R = rpy2rotmZYX(q(4:6));
Bleg = getLegUpperAnchors(model);

rB = R * Bleg;
s = p + rB - model.A;
L = vecnorm(s, 2, 1).';

if any(L <= eps)
    error('sgpIK:ZeroLengthStrut', '存在长度接近零的支链，无法计算单位方向。');
end

u = s ./ L.';

kin = struct();
kin.L = L;
kin.s = s;
kin.u = u;
kin.rB = rB;
kin.R = R;
end

function validateModelGeometry(model)
if ~isstruct(model) || ~isfield(model, 'A') || ~isfield(model, 'B')
    error('sgpIK:MissingGeometry', 'model 必须包含 A 和 B 字段。');
end
validateattributes(model.A, {'double'}, {'real', 'finite', 'size', [3, 6]}, mfilename, 'model.A');
validateattributes(model.B, {'double'}, {'real', 'finite', 'size', [3, 6]}, mfilename, 'model.B');
if isfield(model, 'legMap')
    validateattributes(model.legMap, {'double'}, {'integer', 'vector', 'numel', 6, '>=', 1, '<=', 6}, mfilename, 'model.legMap');
    if numel(unique(model.legMap(:))) ~= 6
        error('sgpIK:InvalidLegMap', 'model.legMap 必须是 1 到 6 的排列。');
    end
end
end

function Bleg = getLegUpperAnchors(model)
if isfield(model, 'legMap')
    Bleg = model.B(:, model.legMap);
else
    Bleg = model.B;
end
end
