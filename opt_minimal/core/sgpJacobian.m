function Jout = sgpJacobian(q, model)
% sgpJacobian - Stewart 平台支链速度雅可比和奇异性指标
%
% 文件用途：
%   计算支链长度速度相对平台速度和广义坐标速度的雅可比矩阵，并给出
%   归一化雅可比的最小奇异值和条件数。
%
% 输入参数：
%   q     [6x1] - [x; y; z; roll; pitch; yaw]。
%   model struct - 至少包含 A、B、Lc。
%
% 输出参数：
%   Jout.Jv       [6x6] - 对空间速度 [v; omega] 的雅可比。
%   Jout.Jq       [6x6] - 对 qdot=[pdot; rpy_dot] 的雅可比。
%   Jout.Jbar     [6x6] - 用空间雅可比 Jv 和特征长度归一化后的雅可比。
%   Jout.JqNorm   [6x6] - 保留给旧绘图/诊断使用的 Jq 归一化结果。
%   Jout.sigmaMin [1x1] - Jbar 的最小奇异值。
%   Jout.condJ    [1x1] - Jbar 的条件数，仅报告，不作为硬约束。
%
% 主要公式：
%   对第 i 条腿，Jv_i = [u_i^T, (r_i x u_i)^T]。
%   omega = E(rpy) * rpy_dot，
%   T = blkdiag(eye(3), E)，
%   Jq = Jv*T。
%
% 与 Stewart 平台轨迹优化的关系：
%   Ldot = Jq*qdot 用于支链速度约束；机构奇异性按任务要求基于空间
%   雅可比 Jv 的归一化矩阵 Jbar 计算，避免 RPY 映射影响奇异性诊断和目标惩罚项。

validateattributes(q, {'double'}, {'real', 'finite', 'vector', 'numel', 6}, mfilename, 'q');
q = q(:);
if isfield(model, 'singularity') && isfield(model.singularity, 'characteristicLength')
    characteristicLength = model.singularity.characteristicLength;
elseif isfield(model, 'Lc')
    characteristicLength = model.Lc;
else
    error('sgpJacobian:InvalidLc', 'model 必须包含 model.Lc 或 model.singularity.characteristicLength。');
end
if ~isscalar(characteristicLength) || ~isfinite(characteristicLength) || characteristicLength <= 0
    error('sgpJacobian:InvalidLc', '雅可比特征长度必须是正的有限标量。');
end

kin = sgpIK(q, model);

Jv = zeros(6, 6);
for legIndex = 1:6
    unitDirection = kin.u(:, legIndex);
    upperJointVector = kin.rB(:, legIndex);

    % 支链长度变化率等于上铰点速度在支链方向上的投影。
    Jv(legIndex, :) = [unitDirection.', cross(upperJointVector, unitDirection).'];
end

E = rpyRateMapZYX(q(4:6));
velocityMap = blkdiag(eye(3), E);
Jq = Jv * velocityMap;

% 按开发任务要求，用 Lc 缩放转动列，避免量纲差异主导奇异值指标。
JqNorm = Jq * diag([1, 1, 1, 1/characteristicLength, 1/characteristicLength, 1/characteristicLength]);
Jbar = Jv * diag([1, 1, 1, 1/characteristicLength, 1/characteristicLength, 1/characteristicLength]);
singularValues = svd(Jbar);

Jout = struct();
Jout.Jv = Jv;
Jout.Jq = Jq;
Jout.JqNorm = JqNorm;
Jout.Jbar = Jbar;
Jout.sigmaMin = min(singularValues);
Jout.condJ = cond(Jbar);
end
