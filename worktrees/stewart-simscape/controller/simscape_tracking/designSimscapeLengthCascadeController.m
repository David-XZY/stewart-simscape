function design = designSimscapeLengthCascadeController(config)
% designSimscapeLengthCascadeController - 逐环整定纯长度串级控制器
arguments
    config struct
end

sampleTime = config.sampleTime;
innerBandwidthHz = config.innerBandwidthHz;
outerBandwidthHz = innerBandwidthHz / config.outerBandwidthRatio;
nyquistHz = 0.5 / sampleTime;
if innerBandwidthHz >= 0.4 * nyquistHz
    error('designSimscapeLengthCascadeController:BandwidthTooHigh', ...
        '内环带宽 %.3f Hz 对 %.6f s 采样周期过高。', innerBandwidthHz, sampleTime);
end

velocityControllers = cell(6, 1);
velocityPlants = cell(6, 1);
velocityClosedLoops = cell(6, 1);
positionClosedLoops = cell(6, 1);
Kpos = zeros(6, 1);
velocityPoles = [];
positionPoles = [];

for legIndex = 1:6
    continuousPlant = tf(config.velocityPlantGain(legIndex), ...
        [config.velocityPlantTimeConstant(legIndex), 1]);
    velocityPlant = c2d(continuousPlant, sampleTime, 'zoh');
    tuneOptions = pidtuneOptions('PhaseMargin', 85);
    tunedVelocityController = pidtune(velocityPlant, 'PIDF', ...
        2 * pi * innerBandwidthHz, tuneOptions);
    tunedVelocityClosedLoop = feedback(velocityPlant * tunedVelocityController, 1);
    tunedPositionPlant = minreal( ...
        tunedVelocityClosedLoop * tf(sampleTime, [1, -1], sampleTime));
    positionController = pidtune(tunedPositionPlant, 'P', 2 * pi * outerBandwidthHz);

    velocityController = tunedVelocityController;
    velocityController.Kp = config.velocityGainScale * velocityController.Kp;
    velocityController.Ki = config.velocityGainScale * velocityController.Ki;
    velocityController.Kd = config.velocityGainScale * velocityController.Kd;
    velocityClosedLoop = feedback(velocityPlant * velocityController, 1);
    positionPlant = minreal(velocityClosedLoop * tf(sampleTime, [1, -1], sampleTime));
    positionController.Kp = config.positionGainScale * positionController.Kp;
    positionClosedLoop = feedback(positionPlant * positionController, 1);

    velocityPlants{legIndex} = velocityPlant;
    velocityControllers{legIndex} = velocityController;
    velocityClosedLoops{legIndex} = velocityClosedLoop;
    positionClosedLoops{legIndex} = positionClosedLoop;
    Kpos(legIndex) = positionController.Kp;
    velocityPoles = [velocityPoles; pole(velocityClosedLoop)]; %#ok<AGROW>
    positionPoles = [positionPoles; pole(positionClosedLoop)]; %#ok<AGROW>
end

stable = all(abs(velocityPoles) < 1 - 1e-7) && ...
    all(abs(positionPoles) < 1 - 1e-7) && all(Kpos > 0);
if ~stable
    error('designSimscapeLengthCascadeController:DesignRejected', ...
        '长度串级自动整定结果未通过离散闭环稳定性检查。');
end

design = struct();
design.sampleTime = sampleTime;
design.innerBandwidthHz = innerBandwidthHz;
design.outerBandwidthHz = outerBandwidthHz;
design.velocityPlants = velocityPlants;
design.velocityControllers = velocityControllers;
design.velocityClosedLoops = velocityClosedLoops;
design.positionClosedLoops = positionClosedLoops;
design.velocityClosedLoopPoles = velocityPoles;
design.positionClosedLoopPoles = positionPoles;
design.Kpos = Kpos;
design.velocityKp = cellfun(@(value) value.Kp, velocityControllers);
design.velocityKi = cellfun(@(value) value.Ki, velocityControllers);
design.velocityKd = cellfun(@(value) value.Kd, velocityControllers);
design.velocityN = cellfun(@(value) 1 / value.Tf, velocityControllers);
design.velocityPhaseMarginDeg = 85;
design.positionGainScale = config.positionGainScale;
design.velocityGainScale = config.velocityGainScale;
design.stable = stable;
end
