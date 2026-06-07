function [z0, initialData] = buildInitialGuessFatropNativeHS(model, scene, disc)
% buildInitialGuessFatropNativeHS - 生成 FATROP-native HS 初值
%
% 本路线沿用现有两阶段五次曲线初值，但只打包 FATROP-native HS 所需变量：
% 每个 stage 的状态 X_k=[q_k;qd_k]，每个区间控制 U_k=[F_k;F_c;F_{k+1}]。
[~, initialData] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
z0 = packFatropNativeHSDecision(initialData.Xnode, initialData.Fnode, initialData.Fmid, disc);
initialData.method = 'FATROP_NATIVE_HS';
end
