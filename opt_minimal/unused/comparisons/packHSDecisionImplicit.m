function z = packHSDecisionImplicit(Xinternal, Anode, Amid, Fnode, Fmid, disc, separator)
% packHSDecisionImplicit - 打包隐式 Hermite-Simpson NLP 决策变量
%
% 用途：
%   将内部节点状态、节点/中点广义加速度和节点/中点支链驱动力按任务书指定顺序
%   打包为 CasADi/IPOPT 使用的列向量；长度由 disc 的节点数、中点数和阶段 1
%   分离证书数量共同决定。
%
% 输入：
%   Xinternal [12x(numNodes-2)] - 不含首尾端点的节点状态 X=[q;qd]
%   Anode     [6xnumNodes]      - 全部节点平台广义加速度 qdd
%   Amid      [6xnumMidpoints]  - 全部中点平台广义加速度 qdd
%   Fnode     [6xnumNodes]      - 全部节点支链轴向驱动力
%   Fmid      [6xnumMidpoints]  - 全部中点支链轴向驱动力
%   disc      struct  - HS 离散配置
%   separator [8xnumStage1CollisionPoints] - 阶段 1 分离证书变量，可为空
%
% 输出：
%   z [numZx1] - 固定顺序 [Xinternal(:); Anode(:); Amid(:); Fnode(:); Fmid(:); separator(:)]
%
% 核心公式：
%   nz = 12*(numNodes-2) + 6*numNodes + 6*numMidpoints
%        + 6*numNodes + 6*numMidpoints + 8*numStage1CollisionPoints。
%
% 优化链路位置：
%   初值生成、CasADi NLP 求解和结果重建共用本函数，避免变量块顺序漂移。

validateattributes(Xinternal, {'double'}, {'real', 'finite', 'size', [12, disc.numNodes-2]}, mfilename, 'Xinternal');
validateattributes(Anode, {'double'}, {'real', 'finite', 'size', [6, disc.numNodes]}, mfilename, 'Anode');
validateattributes(Amid, {'double'}, {'real', 'finite', 'size', [6, disc.numMidpoints]}, mfilename, 'Amid');
validateattributes(Fnode, {'double'}, {'real', 'finite', 'size', [6, disc.numNodes]}, mfilename, 'Fnode');
validateattributes(Fmid, {'double'}, {'real', 'finite', 'size', [6, disc.numMidpoints]}, mfilename, 'Fmid');

if nargin < 7 || isempty(separator)
    separator = zeros(8, 0);
end
numCollisionCertificates = getNumCollisionCertificates(disc);
if numCollisionCertificates > 0
    validateattributes(separator, {'double'}, {'real', 'finite', 'size', [8, numCollisionCertificates]}, ...
        mfilename, 'separator');
end

z = [Xinternal(:); Anode(:); Amid(:); Fnode(:); Fmid(:); separator(:)];
expectedLength = 12*(disc.numNodes-2) + 6*disc.numNodes + 6*disc.numMidpoints + ...
    6*disc.numNodes + 6*disc.numMidpoints;
expectedLength = expectedLength + 8*numCollisionCertificates;
if numel(z) ~= expectedLength
    error('packHSDecisionImplicit:InvalidLength', ...
        '隐式 HS 决策变量数量必须为 %d，当前为 %d。', expectedLength, numel(z));
end
end

function n = getNumCollisionCertificates(disc)
if isfield(disc, 'numCollisionCertificates')
    n = disc.numCollisionCertificates;
elseif isfield(disc, 'numStage1CollisionPoints')
    n = disc.numStage1CollisionPoints;
else
    n = 0;
end
end
