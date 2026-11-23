function [z0, reducedGuess] = buildInitialGuessTwoPhaseReduced(model, scene, disc)
% buildInitialGuessTwoPhaseReduced - 生成 CHSED/DMSED 共用 reduced 初值
%
% 输入：
%   model、scene、disc - 与 CHSID 基线完全相同的模型、场景和离散参数。
%
% 输出：
%   z0 - reduced 决策变量初值，仅包含内部状态、节点力、中点力和分离证书。
%   reducedGuess - 保存从 CHSID 初值继承的轨迹、力、证书和标称参考。
%
% 在实验链路中的作用：
%   CHSED 与 DMSED 从同一条两阶段五次时间律曲线出发，不为某个方法单独提供更优初值。
[zImplicit, implicitGuess] = buildInitialGuessTwoPhaseHSImplicit(model, scene, disc);
[Xnode, ~, ~, Fnode, Fmid, Xinternal, separator] = unpackHSDecisionImplicit(zImplicit, scene, disc);
z0 = packReducedDecisionTwoPhase(Xinternal, Fnode, Fmid, disc, separator);

reducedGuess = struct();
reducedGuess.Xnode = Xnode;
reducedGuess.Xinternal = Xinternal;
reducedGuess.Fnode = Fnode;
reducedGuess.Fmid = Fmid;
reducedGuess.separator = separator;
reducedGuess.nominalStage1 = implicitGuess.nominalStage1;
reducedGuess.source = 'buildInitialGuessTwoPhaseHSImplicit';
end
