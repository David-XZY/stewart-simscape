function [z0, initialData] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc)
% buildInitialGuessTwoPhaseIHSImplicit - 生成 IHSID 中点状态变量化初值
%
% 复用 IHSID 共用的两阶段平滑基础初值，并把采样得到的 QmidCurve/VmidCurve
% 作为独立 Xmid 初值打包，避免把 IHSID 的中点初始化成零。
[~, initialData] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
Xmid = [initialData.QmidCurve; initialData.VmidCurve];
z0 = packIHSDecisionImplicit(initialData.Xnode(:, 2:end-1), Xmid, ...
    initialData.Anode, initialData.Amid, initialData.Fnode, initialData.Fmid, ...
    disc, initialData.separator);
initialData.XmidIHS = Xmid;
end
