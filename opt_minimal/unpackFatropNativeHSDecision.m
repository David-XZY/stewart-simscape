function data = unpackFatropNativeHSDecision(z, scene, disc)
% unpackFatropNativeHSDecision - 解包 FATROP-native HS 决策变量
expectedLength = 12*disc.numNodes + 18*disc.numIntervals + 1;
validateattributes(z, {'double'}, {'real', 'finite', 'vector', 'numel', expectedLength}, mfilename, 'z');
z = z(:);

Xnode = zeros(12, disc.numNodes);
Fleft = zeros(6, disc.numIntervals);
Fmid = zeros(6, disc.numIntervals);
Fright = zeros(6, disc.numIntervals);

cursor = 0;
for intervalIndex = 1:disc.numIntervals
    Xnode(:, intervalIndex) = z(cursor + (1:12));
    cursor = cursor + 12;
    uk = z(cursor + (1:18));
    cursor = cursor + 18;
    Fleft(:, intervalIndex) = uk(1:6);
    Fmid(:, intervalIndex) = uk(7:12);
    Fright(:, intervalIndex) = uk(13:18);
end
Xnode(:, end) = z(cursor + (1:12));
cursor = cursor + 12;
terminalDummy = z(cursor + 1); %#ok<NASGU>

Fnode = zeros(6, disc.numNodes);
Fnode(:, 1:disc.numIntervals) = Fleft;
Fnode(:, end) = Fright(:, end);

data = struct();
data.Xnode = Xnode;
data.Xinternal = Xnode(:, 2:end-1);
data.Fleft = Fleft;
data.Fmid = Fmid;
data.Fright = Fright;
data.Fnode = Fnode;
end
