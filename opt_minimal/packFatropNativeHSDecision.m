function z = packFatropNativeHSDecision(Xnode, Fnode, Fmid, disc)
% packFatropNativeHSDecision - 打包 FATROP-native HS 决策变量
%
% 变量顺序满足 FATROP manual OCP 结构：
%   z = [X_0; U_0; X_1; U_1; ...; X_{N-1}; U_{N-1}; X_N; 0]
% 其中 U_k=[Fleft_k; Fmid_k; Fright_k]。
validateattributes(Xnode, {'double'}, {'real', 'finite', 'size', [12, disc.numNodes]}, mfilename, 'Xnode');
validateattributes(Fnode, {'double'}, {'real', 'finite', 'size', [6, disc.numNodes]}, mfilename, 'Fnode');
validateattributes(Fmid, {'double'}, {'real', 'finite', 'size', [6, disc.numMidpoints]}, mfilename, 'Fmid');

parts = cell(2*disc.numIntervals + 2, 1);
partIndex = 0;
for intervalIndex = 1:disc.numIntervals
    partIndex = partIndex + 1;
    parts{partIndex} = Xnode(:, intervalIndex);
    partIndex = partIndex + 1;
    parts{partIndex} = [Fnode(:, intervalIndex); Fmid(:, intervalIndex); Fnode(:, intervalIndex+1)];
end
partIndex = partIndex + 1;
parts{partIndex} = Xnode(:, end);
partIndex = partIndex + 1;
parts{partIndex} = 0;
z = vertcat(parts{1:partIndex});
end
