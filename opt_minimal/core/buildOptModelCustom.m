function model = buildOptModelCustom()
% buildOptModelCustom - 构建圆柱体两阶段 HS 优化用 6-UCU Stewart 模型
%
% 文件用途：
%   集中配置 Stewart 平台几何、执行器约束、奇异性参数、平台与运动圆柱体合成刚体动力学
%   参数和目标函数归一化尺度。本函数是主优化入口的默认模型来源。
%
% 输入参数：
%   无。
%
% 输出参数：
%   model struct - 包含几何、约束、动力学、目标函数权重和数值阈值。
%
% 核心公式：
%   A_i = rA*[cos(thetaA_i); sin(thetaA_i); 0]，
%   B_i = rB*[cos(thetaB_i); sin(thetaB_i); 0]，
%   legMap 指定第 i 条腿实际连接 A_i -> B_{legMap(i)}。
%
% 在优化链路中的作用：
%   run_01_ihsid_trajectory 调用本函数生成模型。所有路径约束、动力学和
%   绘图函数只读取 model，不在其他文件中散落硬编码平台参数。

model = struct();
model.name = '6-UCU Stewart 圆柱体两阶段 HS 完整动力学模型';

model.rA = 0.75;
model.rB = 0.50;
model.alphaA = deg2rad(15);
model.alphaB = deg2rad(15);
model.phaseA = deg2rad(0);
model.phaseB = deg2rad(60);
model.legMap = [6, 1, 2, 3, 4, 5];

model.A = computeAnchorPoints66(model.rA, model.alphaA, model.phaseA);
model.B = computeAnchorPoints66(model.rB, model.alphaB, model.phaseB);
model.thetaA = atan2(model.A(2, :), model.A(1, :));
model.thetaB = atan2(model.B(2, :), model.B(1, :));

model.zHome = 1.00;
model.qHome = [0; 0; model.zHome; 0; 0; 0];

model.lmin = 1.05 * ones(6, 1);
model.lmax = 2.00 * ones(6, 1);
model.actuator.ldotMax = 0.60 * ones(6, 1);
model.actuator.lddotMax = 1.20 * ones(6, 1);
model.actuator.forceMin = -2000 * ones(6, 1);
model.actuator.forceMax = 2000 * ones(6, 1);

model.Lc = 0.50;
model.singularity.characteristicLength = 0.50;
model.singularity.sigmaMinSafe = 0.10;
model.singularity.condWarning = 20.0;
model.sigmaMinSafe = model.singularity.sigmaMinSafe;
model.condJMax = model.singularity.condWarning;

model.nx = 12;
model.nu = 6;

model.dynamics.includeGravity = true;
model.dynamics.includeLegInertia = false;
model.dynamics.includeFriction = false;
model.dynamics.includeMotorElectricalDynamics = false;
model.dynamics.includeContactForce = false;
model.dynamics.platformMass = 50;
model.dynamics.objectMass = 100;
model.dynamics.totalMass = 150;
model.dynamics.comP = [0; 0; 0.10];
model.dynamics.inertiaAtCOM_P = diag([5.0000, 16.4375, 18.8125]);
model.g = [0; 0; -9.81];

model.objective.weightNominalStage1 = 0.1;
model.objective.weightForceRate = 0.08;
model.objective.weightLegAccel = 0.30;
model.objective.weightSingularity = 0.40;
model.objective.positionDeviationScale = 0.10;
model.objective.attitudeDeviationScale = deg2rad(10);
model.objective.attitudeDeviationWeight = 1.0;
model.objective.forceRateScale = 1000;
model.objective.legAccelScale = 1.2;
model.objective.singularityEpsilon = (0.05 * model.singularity.sigmaMinSafe)^2;
model.objective.singularityScale = model.singularity.sigmaMinSafe^2 / 6;

model.num.rcondMin = 1e-10;
model.num.warnOnIllConditioned = true;
model.rpyOrder = 'ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll)';

homeKin = sgpIK(model.qHome, model);
homeJacobian = sgpJacobian(model.qHome, model);
if any(homeKin.L < model.lmin) || any(homeKin.L > model.lmax)
    error('buildOptModelCustom:HomeLengthOutOfRange', ...
        'qHome 初始腿长不在 [1.05, 2.00] m 范围内，请检查几何参数。');
end
if homeJacobian.sigmaMin < model.singularity.sigmaMinSafe
    error('buildOptModelCustom:HomeSingular', ...
        'qHome 初始 sigmaMin=%.6g，小于安全阈值 %.6g。', ...
        homeJacobian.sigmaMin, model.singularity.sigmaMinSafe);
end
end
