function info = inspectIHSIDManualStructureDimensions(disc)
% inspectIHSIDManualStructureDimensions - 打印 IHSID manual structure 维度说明
%
% FATROP manual 要求 gap 对下一阶段状态是 identity 形式。因此实际求解 builder
% 在 U_k 中增加 24 维右端 Y 副本，原 IHSID mid/Simpson 等式作为阶段 local constraints。
if nargin < 1 || isempty(disc)
    error('inspectIHSIDManualStructureDimensions:MissingDisc', '必须传入 disc。');
end

info = struct();
info.N = disc.numIntervals;
info.YkDescription = 'Y_k = [X_k; A_k; F_k]';
info.nx = 24;
info.UkDescription = ['U_k = [Xmid_k; Amid_k; Fmid_k; sepRoofNode_k; sepRoofMid_k; ', ...
    'sepLeftNode_k; sepLeftMid_k; sepRightMid_k; sepRightNode_k; Yright_k]'];
info.nuBaseWithoutCollision = 24;
info.collisionCertificateSlots = 6;
info.rightNodeCopyDim = 24;
info.nuFixed = 24 + 8*info.collisionCertificateSlots + info.rightNodeCopyDim;
info.gapDescription = 'G_k = Y_{k+1} - Yright_k，原 midConsistency/simpsonDefect 放入 H_k';
info.gapDim = 24;
info.goal = '每个阶段只依赖 Y_k, U_k, Y_{k+1}，且 gap 对 Y_{k+1} 保持 identity。';

info.collisionCertificateCountPerInterval = info.collisionCertificateSlots * ones(1, disc.numIntervals);
info.nuPerInterval = info.nuFixed * ones(1, disc.numIntervals);

fprintf('IHSID manual structure 维度说明：\n');
fprintf('N = %d\n', info.N);
fprintf('%s, nx = %d\n', info.YkDescription, info.nx);
fprintf('%s\n', info.UkDescription);
fprintf('U_k 固定维度 = %d，其中碰撞证书槽位=%d 个，右端 Y 副本=%d 维。\n', ...
    info.nuFixed, info.collisionCertificateSlots, info.rightNodeCopyDim);
fprintf('%s, 维度 = %d\n', info.gapDescription, info.gapDim);
fprintf('%s\n', info.goal);
end
