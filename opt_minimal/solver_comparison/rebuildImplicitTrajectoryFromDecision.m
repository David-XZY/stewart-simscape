function traj = rebuildImplicitTrajectoryFromDecision(z, model, scene, disc)
% rebuildImplicitTrajectoryFromDecision - 从隐式 HS 决策变量重建轨迹结构
%
% 文件用途：
%   将物理决策变量 z 还原为绘图、后验验证、目标分解和 Simscape 导出所需的 traj
%   结构体。该函数只做结果重建，不改变 NLP 数学问题。
%
% 输入：
%   z     - 隐式 HS 决策变量，顺序为 [Xinternal; Anode; Amid; Fnode; Fmid]
%   model - Stewart 模型参数
%   scene - 预对准场景
%   disc  - HS 离散参数
%
% 输出：
%   traj  - 含节点/中点状态、加速度、驱动力和路径诊断量的轨迹结构
%
% 求解链路位置：
%   IPOPT、SQP 两种求解器完成后统一调用本函数，确保后验验证使用同一套重建逻辑。

data = evaluateImplicitTrajectoryNumeric(z, model, scene, disc);
traj = struct();
traj.t = disc.tNode;
traj.tc = disc.tMid;
traj.Xnode = data.Xnode;
traj.Xmid = data.Xmid;
traj.Anode = data.Anode;
traj.Amid = data.Amid;
traj.Unode = data.Fnode;
traj.Umid = data.Fmid;
traj.Q = data.Xnode(1:6, :);
traj.V = data.Xnode(7:12, :);
traj.Qmid = data.Xmid(1:6, :);
traj.Vmid = data.Xmid(7:12, :);
traj.xStart = [scene.q0; scene.qd0];
traj.xEnd = [scene.qPre; scene.qdPre];
traj.nodePoints = data.nodePoint;
traj.midPoints = data.midPoint;
traj.nodeDynResidual = data.nodeDynResidual;
traj.midDynResidual = data.midDynResidual;

traj.L = collectPointField(data.nodePoint, 'L');
traj.Ld = collectPointField(data.nodePoint, 'Ld');
traj.Ldd = collectPointField(data.nodePoint, 'Ldd');
traj.sigmaMin = collectPointScalar(data.nodePoint, 'sigmaMin');
traj.condJ = collectPointScalar(data.nodePoint, 'condJ');
traj.minClearance = collectPointScalar(data.nodePoint, 'minClearance');
traj.Lmid = collectPointField(data.midPoint, 'L');
traj.LdMid = collectPointField(data.midPoint, 'Ld');
traj.LddMid = collectPointField(data.midPoint, 'Ldd');
traj.sigmaMinMid = collectPointScalar(data.midPoint, 'sigmaMin');
traj.condJMid = collectPointScalar(data.midPoint, 'condJ');
traj.minClearanceMid = collectPointScalar(data.midPoint, 'minClearance');
end

function values = collectPointField(points, fieldName)
values = zeros(numel(points{1}.(fieldName)), numel(points));
for index = 1:numel(points)
    values(:, index) = points{index}.(fieldName);
end
end

function values = collectPointScalar(points, fieldName)
values = zeros(1, numel(points));
for index = 1:numel(points)
    values(index) = points{index}.(fieldName);
end
end
