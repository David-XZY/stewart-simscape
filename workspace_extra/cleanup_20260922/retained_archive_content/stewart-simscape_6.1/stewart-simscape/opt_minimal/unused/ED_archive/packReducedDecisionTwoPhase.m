function z = packReducedDecisionTwoPhase(Xinternal, Fnode, Fmid, disc, separator)
% packReducedDecisionTwoPhase - 打包消去加速度后的两阶段决策变量
%
% 输入：
%   Xinternal [12 x (numNodes-2)] - 不含首尾固定端点的状态。
%   Fnode     [6 x numNodes]      - 节点支链力。
%   Fmid      [6 x numMidpoints]  - 中点支链力。
%   separator [8 x numCollisionCertificates] - 碰撞分离证书。
%
% 输出：
%   z - 固定顺序 [Xinternal(:); Fnode(:); Fmid(:); separator(:)]。
validateattributes(Xinternal, {'double'}, {'real', 'finite', 'size', [12, disc.numNodes-2]}, mfilename, 'Xinternal');
validateattributes(Fnode, {'double'}, {'real', 'finite', 'size', [6, disc.numNodes]}, mfilename, 'Fnode');
validateattributes(Fmid, {'double'}, {'real', 'finite', 'size', [6, disc.numMidpoints]}, mfilename, 'Fmid');
validateattributes(separator, {'double'}, {'real', 'finite', 'size', [8, disc.numCollisionCertificates]}, mfilename, 'separator');
z = [Xinternal(:); Fnode(:); Fmid(:); separator(:)];
expectedLength = 12*(disc.numNodes-2) + 6*disc.numNodes + 6*disc.numMidpoints + 8*disc.numCollisionCertificates;
if numel(z) ~= expectedLength
    error('packReducedDecisionTwoPhase:InvalidLength', ...
        'reduced 决策变量长度必须为 %d，当前为 %d。', expectedLength, numel(z));
end
end
