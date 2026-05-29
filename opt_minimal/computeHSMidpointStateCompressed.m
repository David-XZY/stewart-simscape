function Xmid = computeHSMidpointStateCompressed(Xnode, fNode, disc)
% computeHSMidpointStateCompressed - 由端点状态和端点动力学导数计算 HS 中点状态
%
% 文件用途：
%   实现 Hermite-Simpson 中点状态公式。函数名中的 Compressed 是历史命名；
%   当前隐式 NLP 仍复用本公式，由节点状态和节点导数计算几何中点状态。
%
% 输入参数：
%   Xnode [12xnumNodes] - 完整节点状态。
%   fNode [12xnumNodes] - 节点动力学导数 f(Xk,Fk)。
%   disc struct   - HS 离散参数，包含 numIntervals、numMidpoints 和 h。
%
% 输出参数：
%   Xmid [12xnumMidpoints] - 计算得到的区间中点状态。
%
% 核心公式：
%   Xc,k = 0.5*(Xk+Xk+1) + h/8*(fk-fk+1)。
%
% 在优化链路中的作用：
%   evaluateImplicitTrajectoryNumeric 和 buildCasadiImplicitHSNLP 用该公式统一节点到中点的
%   状态重建；目标函数和中点路径/动力学约束也基于该中点状态评价。
validateattributes(Xnode, {'double'}, {'real', 'finite', 'size', [12, disc.numNodes]}, mfilename, 'Xnode');
validateattributes(fNode, {'double'}, {'real', 'finite', 'size', [12, disc.numNodes]}, mfilename, 'fNode');

Xmid = zeros(12, disc.numMidpoints);
for intervalIndex = 1:disc.numIntervals
    Xmid(:, intervalIndex) = 0.5 * (Xnode(:, intervalIndex) + Xnode(:, intervalIndex+1)) + ...
        disc.h/8 * (fNode(:, intervalIndex) - fNode(:, intervalIndex+1));
end
end
