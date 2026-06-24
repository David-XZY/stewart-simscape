function test_41_lqi_pose_force_design_contract
% test_41_lqi_pose_force_design_contract - 验证 LQI 位姿力反馈设计接口
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
addpath(fullfile(controllerRoot, 'simscape_tracking'));

cartesianPlant = ss(tf(1, [1 0 0]) * eye(6));
config = struct();
config.bandwidthHz = 15;
config.gainScale = 0.45;
config.rotationGainScale = 1.4;
config.lqiTranslationScale = 3e-2;
config.lqiRotationScale = deg2rad(1);
config.lqiVelocityScale = 5e-3;
config.lqiIntegralScale = 2e-1;
config.lqiControlScale = 10;
config.lqiIntegralLeakHz = 1;
config.lqiDerivativeFilterTime = 1 / (2 * pi * config.bandwidthHz);
config.lqiAntiWindupTimeConstant = 0.02;

lqiDesign = designLqiPoseForceController(cartesianPlant, 0.5, config);

assert(lqiDesign.feedbackLaw == "lqi-output");
assert(lqiDesign.integratorCount == 6);
assert(lqiDesign.feedbackInputCount == 12);
assert(isequal(size(lqiDesign.Kx), [6, 12]));
assert(isa(lqiDesign.Kx, 'ss'));
assert(lqiDesign.stable);
assert(numel(lqiDesign.closedLoopPoles) > 0);
end
