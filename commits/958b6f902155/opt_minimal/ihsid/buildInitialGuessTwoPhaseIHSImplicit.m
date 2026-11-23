function [z0, initialData] = buildInitialGuessTwoPhaseIHSImplicit(model, scene, disc, external)
% buildInitialGuessTwoPhaseIHSImplicit - 生成 IHSID 中点状态变量化初值
%
% 复用 IHSID 共用的两阶段平滑基础初值，并把采样得到的 QmidCurve/VmidCurve
% 作为独立 Xmid 初值打包，避免把 IHSID 的中点初始化成零。
[~, initialData] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
if nargin>=4 && ~isempty(external)
    required={'Xnode','Xmid','Anode','Amid','Fnode','Fmid','separator'};
    expected={[12,disc.numNodes],[12,disc.numMidpoints],[6,disc.numNodes], ...
        [6,disc.numMidpoints],[6,disc.numNodes],[6,disc.numMidpoints],[8,disc.numCollisionCertificates]};
    for fieldIndex=1:numel(required)
        name=required{fieldIndex};
        assert(isfield(external,name) && isequal(size(external.(name)),expected{fieldIndex}) && ...
            all(isfinite(external.(name)),'all'),'ABFD:InitialGuessShape','Invalid external %s.',name);
        initialData.(name)=external.(name);
    end
    assert(norm(external.Xnode(:,1)-[scene.q0;scene.qd0])<1e-8 && ...
        norm(external.Xnode(:,end)-[scene.qGoal;scene.qdGoal])<1e-8, ...
        'ABFD:InitialGuessBoundary','External endpoints violate task boundary.');
    initialData.QmidCurve=external.Xmid(1:6,:);
    initialData.VmidCurve=external.Xmid(7:12,:);
end
Xmid = [initialData.QmidCurve; initialData.VmidCurve];
z0 = packIHSDecisionImplicit(initialData.Xnode(:, 2:end-1), Xmid, ...
    initialData.Anode, initialData.Amid, initialData.Fnode, initialData.Fmid, ...
    disc, initialData.separator);
initialData.XmidIHS = Xmid;
end
