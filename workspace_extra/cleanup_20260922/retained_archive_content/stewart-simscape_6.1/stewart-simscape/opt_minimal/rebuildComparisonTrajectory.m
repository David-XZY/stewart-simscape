function traj = rebuildComparisonTrajectory(data, scene, disc)
% rebuildComparisonTrajectory - 将数值重建 data 转为绘图和密集验证使用的 traj
%
% 输入：
%   data - evaluateImplicit/ReducedHS/DMS 产生的轨迹诊断结构。
%   scene、disc - 场景和离散信息。
%
% 输出：
%   traj - 与 run_01 中 traj 字段兼容的结构体。
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
traj.xWaypoint = [scene.qWaypoint; data.Xnode(7:12, disc.waypointNodeIndex)];
traj.xEnd = [scene.qGoal; scene.qdGoal];
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
traj.collisionDistances = collectPointField(data.nodePoint, 'collisionDistances');
traj.Lmid = collectPointField(data.midPoint, 'L');
traj.LdMid = collectPointField(data.midPoint, 'Ld');
traj.LddMid = collectPointField(data.midPoint, 'Ldd');
traj.sigmaMinMid = collectPointScalar(data.midPoint, 'sigmaMin');
traj.condJMid = collectPointScalar(data.midPoint, 'condJ');
traj.minClearanceMid = collectPointScalar(data.midPoint, 'minClearance');
traj.collisionDistancesMid = collectPointField(data.midPoint, 'collisionDistances');
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
