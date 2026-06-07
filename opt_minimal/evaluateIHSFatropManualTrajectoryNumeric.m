function data = evaluateIHSFatropManualTrajectoryNumeric(zManual, model, scene, disc)
% evaluateIHSFatropManualTrajectoryNumeric - FATROP manual 解回现有 IHSID 评价链路
manualData = unpackIHSFatropManualDecision(zManual, scene, disc);
zIHS = packIHSDecisionImplicit(manualData.Xinternal, manualData.Xmid, manualData.Anode, ...
    manualData.Amid, manualData.Fnode, manualData.Fmid, disc, manualData.separator);
data = evaluateImplicitIHSTrajectoryNumeric(zIHS, model, scene, disc);
data.zIHS = zIHS;
data.zManual = zManual(:);
data.method = 'IHSID-FATROP-manual';
end
