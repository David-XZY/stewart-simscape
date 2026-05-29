function J = costHSDynamic(z, model, scene, disc)
% costHSDynamic - 状态/控制 HS 预对准轨迹目标函数
%
% 文件用途：
%   使用 Simpson 积分计算驱动力归一化平方和腿加速度归一化平方目标。
%
% 输入参数：
%   z [738x1] - HS 决策变量 [Xnode;Xmid;Unode;Umid]。
%   model struct - Stewart 模型与目标权重。
%   scene struct - 预对准场景。
%   disc struct - 离散参数。
%
% 输出参数：
%   J [1x1] - 标量目标值。
%
% 核心公式：
%   L=Σ(F_i/2000)^2+0.10Σ(Ldd_i/1.20)^2，
%   J=Σ h/6*(L_k+4L_mid+L_{k+1})。
%
% 在优化链路中的作用：
%   fmincon 最小化本目标；不含终点误差、碰撞软惩罚或参考跟踪项。

[Xnode, Xmid, Unode, Umid] = unpackHSDecision(z, disc);

stageNode = zeros(1, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    point = evaluatePathConstraintsAtPoint(Xnode(:, nodeIndex), Unode(:, nodeIndex), model, scene);
    stageNode(nodeIndex) = stageCost(point, model);
end

J = 0;
for intervalIndex = 1:disc.numIntervals
    pointMid = evaluatePathConstraintsAtPoint(Xmid(:, intervalIndex), Umid(:, intervalIndex), model, scene);
    stageMid = stageCost(pointMid, model);
    J = J + disc.h/6 * (stageNode(intervalIndex) + 4*stageMid + stageNode(intervalIndex+1));
end
end

function value = stageCost(point, model)
value = model.objective.weightForce * sum((point.F ./ model.objective.forceScale).^2) + ...
    model.objective.weightLegAccel * sum((point.Ldd ./ model.objective.legAccelScale).^2);
end
