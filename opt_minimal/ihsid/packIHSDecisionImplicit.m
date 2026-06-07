function z = packIHSDecisionImplicit(Xinternal, Xmid, Anode, Amid, Fnode, Fmid, disc, separator)
% packIHSDecisionImplicit - 打包 IHSID 隐式 HS 决策变量
%
% Xmid 作为独立优化变量，顺序固定为
% [Xinternal; Xmid; Anode; Amid; Fnode; Fmid; separator]。
validateattributes(Xinternal, {'double'}, {'real', 'finite', 'size', [12, disc.numNodes-2]}, mfilename, 'Xinternal');
validateattributes(Xmid, {'double'}, {'real', 'finite', 'size', [12, disc.numMidpoints]}, mfilename, 'Xmid');
validateattributes(Anode, {'double'}, {'real', 'finite', 'size', [6, disc.numNodes]}, mfilename, 'Anode');
validateattributes(Amid, {'double'}, {'real', 'finite', 'size', [6, disc.numMidpoints]}, mfilename, 'Amid');
validateattributes(Fnode, {'double'}, {'real', 'finite', 'size', [6, disc.numNodes]}, mfilename, 'Fnode');
validateattributes(Fmid, {'double'}, {'real', 'finite', 'size', [6, disc.numMidpoints]}, mfilename, 'Fmid');

if nargin < 8 || isempty(separator)
    separator = zeros(8, 0);
end
numCollisionCertificates = getNumCollisionCertificates(disc);
if numCollisionCertificates > 0
    validateattributes(separator, {'double'}, {'real', 'finite', 'size', [8, numCollisionCertificates]}, ...
        mfilename, 'separator');
end

z = [Xinternal(:); Xmid(:); Anode(:); Amid(:); Fnode(:); Fmid(:); separator(:)];
expectedLength = 12*(disc.numNodes-2) + 12*disc.numMidpoints + 6*disc.numNodes + ...
    6*disc.numMidpoints + 6*disc.numNodes + 6*disc.numMidpoints + 8*numCollisionCertificates;
if numel(z) ~= expectedLength
    error('packIHSDecisionImplicit:InvalidLength', ...
        'IHSID 决策变量数量必须为 %d，当前为 %d。', expectedLength, numel(z));
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
