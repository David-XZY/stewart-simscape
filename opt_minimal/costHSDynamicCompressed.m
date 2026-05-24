function J = costHSDynamicCompressed(z, model, scene, disc)
% costHSDynamicCompressed - 压缩 HS 预对准轨迹目标函数
%
% 文件用途：
%   在节点和由压缩 HS 公式计算出的中点上，按 Simpson 积分累计归一化驱动力平方与
%   归一化腿加速度平方。
%
% 输入参数：
%   z [474x1]  - 压缩 HS 决策变量。
%   model      - 包含 objective.forceScale、legAccelScale 和权重。
%   scene      - 预对准场景。
%   disc       - HS 离散参数。
%
% 输出参数：
%   J [1x1] - 标量目标函数值。
%
% 核心公式：
%   L=sum((F/2000)^2)+0.10*sum((Ldd/1.20)^2)，
%   J=sum_k h/6*(Lk+4*Lc+Lk+1)。
%
% 在优化链路中的作用：
%   fmincon 的目标函数；不包含终点误差、参考跟踪、平台加速度或碰撞软惩罚。
data = evaluateCompressedTrajectory(z, model, scene, disc);

stageNode = zeros(1, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    stageNode(nodeIndex) = runningCost(data.Fnode(:, nodeIndex), data.nodePoint{nodeIndex}.Ldd, model);
end

J = 0;
for intervalIndex = 1:disc.numIntervals
    stageMid = runningCost(data.Fmid(:, intervalIndex), data.midPoint{intervalIndex}.Ldd, model);
    J = J + disc.h/6 * (stageNode(intervalIndex) + 4*stageMid + stageNode(intervalIndex+1));
end
end

function value = runningCost(F, Ldd, model)
value = model.objective.weightForce * sum((F ./ model.objective.forceScale).^2) + ...
    model.objective.weightLegAccel * sum((Ldd ./ model.objective.legAccelScale).^2);
end
