function [scale, scaleSummary] = buildImplicitHSVariableScale(z0, model, scene, disc)
% buildImplicitHSVariableScale - 构造 fmincon-SQP 使用的变量尺度向量
%
% 文件用途：
%   为数值变量 y 构造 z = scale .* y 的尺度向量。该尺度只改变 SQP 的数值表示，
%   所有目标函数和约束仍在物理变量 z 上评价，不改变原始优化问题。
%
% 输入：
%   z0    - 当前隐式 HS 初值
%   model - 含 objective.forceScale、legAccelScale 等已有尺度参数
%   scene - 场景端点，用于解包初值
%   disc  - HS 离散参数
%
% 输出：
%   scale        - 与 z 等长的正尺度向量
%   scaleSummary - 各变量块尺度来源和范围摘要
%
% 求解链路位置：
%   solveImplicitHSSQP 在启用 useVariableScaling 时调用本函数，再交给
%   buildFminconSQPAdapterFromCasadi 应用链式法则。

[Xnode, Anode, Amid, Fnode, Fmid, Xinternal] = unpackHSDecisionImplicit(z0, scene, disc); %#ok<ASGLU>

stateLower = [1; 1; 1; 0.25; 0.25; 0.25; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5];
stateScale = max(max(abs(Xnode), [], 2), stateLower);

accelTypical = max([abs(Anode(:)); abs(Amid(:)); model.objective.legAccelScale; 0.1]);
accelScale = accelTypical * ones(6, 1);

forceScale = model.objective.forceScale * ones(6, 1);

scaleXinternal = repmat(stateScale, disc.numNodes - 2, 1);
scaleAnode = repmat(accelScale, disc.numNodes, 1);
scaleAmid = repmat(accelScale, disc.numMidpoints, 1);
scaleFnode = repmat(forceScale, disc.numNodes, 1);
scaleFmid = repmat(forceScale, disc.numMidpoints, 1);

scale = [scaleXinternal; scaleAnode; scaleAmid; scaleFnode; scaleFmid];
scale = max(scale(:), eps);

scaleSummary = struct();
scaleSummary.stateScale = stateScale;
scaleSummary.accelScale = accelScale;
scaleSummary.forceScale = forceScale;
scaleSummary.minScale = min(scale);
scaleSummary.maxScale = max(scale);
scaleSummary.description = ['Xinternal uses row-wise state magnitudes with lower bounds; ', ...
    'Anode/Amid use initial acceleration and legAccelScale; Fnode/Fmid use forceScale.'];
end
