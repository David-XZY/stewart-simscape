function data = unpackIHSFatropManualDecision(zManual, scene, disc)
% unpackIHSFatropManualDecision - 解包 FATROP manual 阶段顺序并恢复现有 IHSID 数据结构
N = disc.numIntervals;
expectedLength = N * (24 + 96) + 24 + terminalNu(disc);
validateattributes(zManual, {'double'}, {'real', 'finite', 'vector', 'numel', expectedLength}, mfilename, 'zManual');
zManual = zManual(:);

Xnode = zeros(12, N+1);
Anode = zeros(6, N+1);
Fnode = zeros(6, N+1);
Xmid = zeros(12, N);
Amid = zeros(6, N);
Fmid = zeros(6, N);
separator = zeros(8, disc.numCollisionCertificates);

cursor = 0;
for nodeIndex = 1:N
    yk = zManual(cursor + (1:24));
    cursor = cursor + 24;
    uk = zManual(cursor + (1:96));
    cursor = cursor + 96;

    Xnode(:, nodeIndex) = yk(1:12);
    Anode(:, nodeIndex) = yk(13:18);
    Fnode(:, nodeIndex) = yk(19:24);
    Xmid(:, nodeIndex) = uk(1:12);
    Amid(:, nodeIndex) = uk(13:18);
    Fmid(:, nodeIndex) = uk(19:24);

    separator = setSeparatorIfUsed(separator, roofNodeSepIndex(nodeIndex, disc), uk(25:32));
    separator = setSeparatorIfUsed(separator, roofMidSepIndex(nodeIndex, disc), uk(33:40));
    separator = setSeparatorIfUsed(separator, leftNodeSepIndex(nodeIndex, disc), uk(41:48));
    separator = setSeparatorIfUsed(separator, leftMidSepIndex(nodeIndex, disc), uk(49:56));
    separator = setSeparatorIfUsed(separator, rightMidSepIndex(nodeIndex, disc), uk(57:64));
    separator = setSeparatorIfUsed(separator, rightNodeSepIndex(nodeIndex, disc), uk(65:72));
end

terminalNode = N + 1;
yk = zManual(cursor + (1:24));
cursor = cursor + 24;
ukTerminal = zManual(cursor + (1:terminalNu(disc)));
Xnode(:, terminalNode) = yk(1:12);
Anode(:, terminalNode) = yk(13:18);
Fnode(:, terminalNode) = yk(19:24);
separator = unpackTerminalSeparator(separator, ukTerminal, terminalNode, disc);

data = struct();
data.Xnode = Xnode;
data.Xinternal = Xnode(:, 2:end-1);
data.Xmid = Xmid;
data.Anode = Anode;
data.Amid = Amid;
data.Fnode = Fnode;
data.Fmid = Fmid;
data.separator = separator;
end

function separator = unpackTerminalSeparator(separator, ukTerminal, nodeIndex, disc)
cursor = 0;
roofIndex = roofNodeSepIndex(nodeIndex, disc);
if roofIndex > 0
    separator = setSeparatorIfUsed(separator, roofIndex, ukTerminal(cursor + (1:8)));
    cursor = cursor + 8;
end
separator = setSeparatorIfUsed(separator, leftNodeSepIndex(nodeIndex, disc), ukTerminal(cursor + (1:8)));
cursor = cursor + 8;
separator = setSeparatorIfUsed(separator, rightNodeSepIndex(nodeIndex, disc), ukTerminal(cursor + (1:8)));
end

function n = terminalNu(disc)
n = 16;
if roofNodeSepIndex(disc.numIntervals + 1, disc) > 0
    n = n + 8;
end
end

function separator = setSeparatorIfUsed(separator, index, value)
if index > 0
    separator(:, index) = value;
end
end

function index = roofNodeSepIndex(nodeIndex, disc)
if nodeIndex <= disc.numIntervalsApproach + 1
    index = nodeIndex;
else
    index = 0;
end
end

function index = roofMidSepIndex(intervalIndex, disc)
if intervalIndex <= disc.numIntervalsApproach
    index = disc.numIntervalsApproach + 1 + intervalIndex;
else
    index = 0;
end
end

function index = leftNodeSepIndex(nodeIndex, disc)
index = disc.numStage1CollisionPoints + nodeIndex;
end

function index = rightNodeSepIndex(nodeIndex, disc)
index = disc.numStage1CollisionPoints + disc.numAllCollisionPoints + nodeIndex;
end

function index = leftMidSepIndex(intervalIndex, disc)
index = disc.numStage1CollisionPoints + disc.numNodes + intervalIndex;
end

function index = rightMidSepIndex(intervalIndex, disc)
index = disc.numStage1CollisionPoints + disc.numAllCollisionPoints + disc.numNodes + intervalIndex;
end
