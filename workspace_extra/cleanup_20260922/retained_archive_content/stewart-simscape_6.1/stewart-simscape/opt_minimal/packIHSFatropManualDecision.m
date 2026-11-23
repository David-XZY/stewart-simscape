function zManual = packIHSFatropManualDecision(zIHS, scene, disc)
% packIHSFatropManualDecision - 将现有 IHSID 决策变量重排为 FATROP manual 阶段顺序
%
% 前 N 个区间使用 U_k=[Xmid; Amid; Fmid; sep slots; Yright]。终端阶段不再
% 附加完整 96 维 U_N，只保留终端节点碰撞分离证书所需的最小占位变量，避免
% 改变终端节点路径约束含义。
[Xnode, Xmid, Anode, Amid, Fnode, Fmid, ~, separator] = unpackIHSDecisionImplicit(zIHS, scene, disc);

N = disc.numIntervals;
zParts = cell(2*N + 2, 1);
partIndex = 0;
for nodeIndex = 1:N
    yk = [Xnode(:, nodeIndex); Anode(:, nodeIndex); Fnode(:, nodeIndex)];
    uk = zeros(96, 1);
    uk(1:12) = Xmid(:, nodeIndex);
    uk(13:18) = Amid(:, nodeIndex);
    uk(19:24) = Fmid(:, nodeIndex);
    uk(25:32) = getSeparatorOrZero(separator, roofNodeSepIndex(nodeIndex, disc));
    uk(33:40) = getSeparatorOrZero(separator, roofMidSepIndex(nodeIndex, disc));
    uk(41:48) = getSeparatorOrZero(separator, leftNodeSepIndex(nodeIndex, disc));
    uk(49:56) = getSeparatorOrZero(separator, leftMidSepIndex(nodeIndex, disc));
    uk(57:64) = getSeparatorOrZero(separator, rightMidSepIndex(nodeIndex, disc));
    uk(65:72) = getSeparatorOrZero(separator, rightNodeSepIndex(nodeIndex, disc));
    uk(73:96) = [Xnode(:, nodeIndex+1); Anode(:, nodeIndex+1); Fnode(:, nodeIndex+1)];

    partIndex = partIndex + 1;
    zParts{partIndex} = yk;
    partIndex = partIndex + 1;
    zParts{partIndex} = uk;
end

terminalNode = N + 1;
partIndex = partIndex + 1;
zParts{partIndex} = [Xnode(:, terminalNode); Anode(:, terminalNode); Fnode(:, terminalNode)];
partIndex = partIndex + 1;
zParts{partIndex} = packTerminalSeparator(separator, terminalNode, disc);
zManual = vertcat(zParts{1:partIndex});
end

function ukTerminal = packTerminalSeparator(separator, nodeIndex, disc)
ukTerminal = [getSeparatorOrZero(separator, leftNodeSepIndex(nodeIndex, disc)); ...
              getSeparatorOrZero(separator, rightNodeSepIndex(nodeIndex, disc))];
roofIndex = roofNodeSepIndex(nodeIndex, disc);
if roofIndex > 0
    ukTerminal = [getSeparatorOrZero(separator, roofIndex); ukTerminal];
end
end

function sep = getSeparatorOrZero(separator, index)
if index > 0
    sep = separator(:, index);
else
    sep = zeros(8, 1);
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
