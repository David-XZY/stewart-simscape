function design = designSimscapePoseForceController(modelName, model, config)
% designSimscapePoseForceController - 整定力输入笛卡尔位姿动态反馈控制器
arguments
    modelName char
    model struct
    config struct
end

options = linearizeOptions;
options.SampleTime = 0;
io(1) = linio([modelName, '/Controller'], 1, 'openinput');
io(2) = linio([modelName, '/Relative Motion Sensor'], 1, 'openoutput');
plant = linearize(modelName, io, options);
if ~isequal(size(plant), [6, 6])
    error('designSimscapePoseForceController:InvalidPlantSize', ...
        '力到位姿线性化对象必须为 6x6，当前为 %dx%d。', size(plant, 1), size(plant, 2));
end

jacobian = sgpJacobian(model.qHome, model).Jv;
forceMapping = jacobian.' \ eye(6);
cartesianPlant = minreal(plant * forceMapping);
feedbackLaw = string(config.feedbackLaw);
lqiDesign = designLqiPoseForceController(cartesianPlant, model.Lc, config);
Kx = lqiDesign.Kx;
antiWindupInputMap = blkdiag(eye(6), jacobian.');
K = ss(forceMapping) * Kx * ss(antiWindupInputMap);
closedLoop = [];
closedLoopPoles = lqiDesign.closedLoopPoles;
stable = lqiDesign.stable;

design = struct();
design.feedbackLaw = feedbackLaw;
design.bandwidthHz = config.bandwidthHz;
design.gainScale = config.gainScale;
design.rotationGainScale = config.rotationGainScale;
design.plant = plant;
design.jacobian = jacobian;
design.forceMapping = forceMapping;
design.antiWindupInputMap = antiWindupInputMap;
design.cartesianPlant = cartesianPlant;
design.Kx = Kx;
design.K = K;
design.lqi = lqiDesign;
design.integratorCount = lqiDesign.integratorCount;
design.feedbackInputCount = lqiDesign.feedbackInputCount;
design.closedLoop = closedLoop;
design.closedLoopPoles = closedLoopPoles;
design.stable = stable;

if ~stable
    error('designSimscapePoseForceController:DesignRejected', ...
        '力输入位姿反馈闭环未通过线性稳定性检查。');
end
end
