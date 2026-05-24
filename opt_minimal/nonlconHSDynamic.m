function [c, ceq] = nonlconHSDynamic(z, model, scene, disc)
% nonlconHSDynamic - 状态/控制 HS 完整 NLP 非线性约束
%
% 文件用途：
%   生成 41 个检查点的路径不等式约束，以及 20 个区间的显式中点和
%   Simpson 积分等式约束。
%
% 输入参数：
%   z [738x1] - HS 决策变量。
%   model struct - Stewart 模型。
%   scene struct - 预对准场景和边界状态。
%   disc struct - 离散参数。
%
% 输出参数：
%   c [2706x1] - 非线性路径约束，全部为 c<=0。
%   ceq [504x1] - 边界状态和 HS 配点等式残差。
%
% 核心公式：
%   Xmid-(Xk+Xk1)/2-h/8*(fk-fk1)=0，
%   Xk1-Xk-h/6*(fk+4*fmid+fk1)=0。
%
% 在优化链路中的作用：
%   fmincon 使用本函数保证状态/控制轨迹满足动力学、机构和碰撞硬约束。

[Xnode, Xmid, Unode, Umid] = unpackHSDecision(z, disc);

c = [];
nodePoint = cell(1, disc.numNodes);
for nodeIndex = 1:disc.numNodes
    nodePoint{nodeIndex} = evaluatePathConstraintsAtPoint(Xnode(:, nodeIndex), Unode(:, nodeIndex), model, scene);
    c = [c; nodePoint{nodeIndex}.cPath]; %#ok<AGROW>
end

xStart = [scene.q0; scene.qd0];
xEnd = [scene.qPre; scene.qdPre];
ceq = [Xnode(:, 1) - xStart;
       Xnode(:, end) - xEnd];

for intervalIndex = 1:disc.numIntervals
    midPoint = evaluatePathConstraintsAtPoint(Xmid(:, intervalIndex), Umid(:, intervalIndex), model, scene);
    c = [c; midPoint.cPath]; %#ok<AGROW>

    fk = nodePoint{intervalIndex}.xdot;
    fk1 = nodePoint{intervalIndex+1}.xdot;
    fmid = midPoint.xdot;

    ceqMid = Xmid(:, intervalIndex) - 0.5*(Xnode(:, intervalIndex) + Xnode(:, intervalIndex+1)) - ...
        disc.h/8 * (fk - fk1);
    ceqStep = Xnode(:, intervalIndex+1) - Xnode(:, intervalIndex) - ...
        disc.h/6 * (fk + 4*fmid + fk1);
    ceq = [ceq; ceqMid; ceqStep]; %#ok<AGROW>
end
end
