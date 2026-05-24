function [c, ceq] = nonlconHSDynamicCompressed(z, model, scene, disc)
% nonlconHSDynamicCompressed - 压缩 HS 完整 NLP 非线性约束
%
% 文件用途：
%   返回所有节点和中点的路径硬约束，以及 20 个区间的压缩 Hermite-Simpson 动力学
%   缺陷。端点状态由解包函数补入常数，不再作为等式约束返回。
%
% 输入参数：
%   z [474x1]  - 压缩 HS 决策变量。
%   model      - Stewart 模型和约束参数。
%   scene      - 预对准场景和碰撞几何。
%   disc       - HS 离散参数。
%
% 输出参数：
%   c [2706x1]   - 路径不等式，全部满足 c<=0。
%   ceq [240x1] - 压缩 HS 动力学缺陷，全部满足 ceq=0。
%
% 核心公式：
%   Xc,k = 0.5*(Xk+Xk+1)+h/8*(fk-fk+1)，
%   ceq_k = Xk+1-Xk-h/6*(fk+4*fc+fk+1)。
%
% 在优化链路中的作用：
%   fmincon 使用本函数同时保证动力学、执行器运动学、奇异性和碰撞安全硬约束。
data = evaluateCompressedTrajectory(z, model, scene, disc);

constraintsPerPoint = numel(data.nodePoint{1}.cPath);
expectedC = constraintsPerPoint * (disc.numNodes + disc.numMidpoints);
c = zeros(expectedC, 1);
cursor = 0;
for nodeIndex = 1:disc.numNodes
    block = cursor + (1:constraintsPerPoint);
    c(block) = data.nodePoint{nodeIndex}.cPath;
    cursor = cursor + constraintsPerPoint;
end
for midIndex = 1:disc.numMidpoints
    block = cursor + (1:constraintsPerPoint);
    c(block) = data.midPoint{midIndex}.cPath;
    cursor = cursor + constraintsPerPoint;
end

ceq = zeros(12 * disc.numIntervals, 1);
for intervalIndex = 1:disc.numIntervals
    block = (intervalIndex - 1) * 12 + (1:12);
    ceq(block) = data.Xnode(:, intervalIndex+1) - data.Xnode(:, intervalIndex) - ...
        disc.h/6 * (data.fNode(:, intervalIndex) + 4*data.fMid(:, intervalIndex) + data.fNode(:, intervalIndex+1));
end

if numel(c) ~= 2706
    error('nonlconHSDynamicCompressed:InvalidInequalityCount', ...
        '路径不等式数量必须为 2706，当前为 %d。', numel(c));
end
if numel(ceq) ~= 240
    error('nonlconHSDynamicCompressed:InvalidEqualityCount', ...
        'compressed HS 等式数量必须为 240，当前为 %d。', numel(ceq));
end
end
