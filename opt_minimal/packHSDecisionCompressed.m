function z = packHSDecisionCompressed(Xinternal, Fnode, Fmid, disc)
% packHSDecisionCompressed - 打包压缩 Hermite-Simpson 决策变量
%
% 文件用途：
%   将内部节点状态、全部节点控制力和全部中点控制力按固定顺序打包成 fmincon 使用
%   的 474 维列向量。起点和终点状态是场景常数，不进入决策变量。
%
% 输入参数：
%   Xinternal [12x19] - 内部节点状态 X1,...,X19，X=[q;qd]。
%   Fnode     [6x21]  - 节点支链轴向驱动力 F0,...,F20。
%   Fmid      [6x20]  - 区间中点支链轴向驱动力 Fc0,...,Fc19。
%   disc struct       - HS 离散参数，至少包含 numNodes 和 numMidpoints。
%
% 输出参数：
%   z [474x1] - 压缩 HS 决策变量，顺序为 [Xinternal(:);Fnode(:);Fmid(:)]。
%
% 核心公式：
%   nz = 12*(21-2) + 6*21 + 6*20 = 474。
%
% 在优化链路中的作用：
%   主脚本和初值生成函数用本函数统一变量顺序，避免与旧 Q/V/A/Ac 或 738 变量接口混用。
validateattributes(Xinternal, {'double'}, {'real', 'finite', 'size', [12, disc.numNodes-2]}, mfilename, 'Xinternal');
validateattributes(Fnode, {'double'}, {'real', 'finite', 'size', [6, disc.numNodes]}, mfilename, 'Fnode');
validateattributes(Fmid, {'double'}, {'real', 'finite', 'size', [6, disc.numMidpoints]}, mfilename, 'Fmid');

z = [Xinternal(:); Fnode(:); Fmid(:)];
if numel(z) ~= 474
    error('packHSDecisionCompressed:InvalidLength', ...
        'compressed HS 决策变量数量必须为 474，当前为 %d。', numel(z));
end
end
