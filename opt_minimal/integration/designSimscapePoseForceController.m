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
cartesianControllers = cell(1, 6);
for axisIndex = 1:6
    cartesianControllers{axisIndex} = pidtune( ...
        cartesianPlant(axisIndex, axisIndex), 'PIDF', 2 * pi * config.bandwidthHz);
end

axisScale = config.gainScale * ones(6, 1);
axisScale(4:6) = axisScale(4:6) * config.rotationGainScale;
Kx = ss(diag(axisScale)) * ss(blkdiag(cartesianControllers{:}));
K = ss(forceMapping) * Kx;
closedLoop = feedback(plant * K, eye(6));
closedLoopPoles = pole(closedLoop);
stable = all(real(closedLoopPoles) < -1e-7);

design = struct();
design.bandwidthHz = config.bandwidthHz;
design.gainScale = config.gainScale;
design.rotationGainScale = config.rotationGainScale;
design.plant = plant;
design.jacobian = jacobian;
design.forceMapping = forceMapping;
design.cartesianPlant = cartesianPlant;
design.cartesianControllers = cartesianControllers;
design.Kx = Kx;
design.K = K;
design.closedLoop = closedLoop;
design.closedLoopPoles = closedLoopPoles;
design.stable = stable;

if ~stable
    error('designSimscapePoseForceController:DesignRejected', ...
        '力输入位姿反馈闭环未通过线性稳定性检查。');
end
end
