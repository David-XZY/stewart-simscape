function Xmid = computeHSMidpointStateCompressed(Xnode, fNode, disc)
% computeHSMidpointStateCompressed - 由端点状态和端点动力学导数计算 HS 中点状态
%
% 文件用途：
%   实现压缩 Hermite-Simpson 形式中的中点状态公式。Xmid 不作为优化变量，而是在
%   每次评价 cost/nonlcon 时由端点状态和显式动力学导数直接生成。
%
% 输入参数：
%   Xnode [12x21] - 完整节点状态。
%   fNode [12x21] - 节点动力学导数 f(Xk,Fk)。
%   disc struct   - HS 离散参数，包含 numIntervals、numMidpoints 和 h。
%
% 输出参数：
%   Xmid [12x20] - 计算得到的区间中点状态。
%
% 核心公式：
%   Xc,k = 0.5*(Xk+Xk+1) + h/8*(fk-fk+1)。
%
% 在优化链路中的作用：
%   nonlcon 用该中点评价中点动力学和路径硬约束；目标函数也在该中点上做 Simpson 积分。
validateattributes(Xnode, {'double'}, {'real', 'finite', 'size', [12, disc.numNodes]}, mfilename, 'Xnode');
validateattributes(fNode, {'double'}, {'real', 'finite', 'size', [12, disc.numNodes]}, mfilename, 'fNode');

Xmid = zeros(12, disc.numMidpoints);
for intervalIndex = 1:disc.numIntervals
    Xmid(:, intervalIndex) = 0.5 * (Xnode(:, intervalIndex) + Xnode(:, intervalIndex+1)) + ...
        disc.h/8 * (fNode(:, intervalIndex) - fNode(:, intervalIndex+1));
end
end
