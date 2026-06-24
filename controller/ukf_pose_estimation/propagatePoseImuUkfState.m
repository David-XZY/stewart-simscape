function nextState = propagatePoseImuUkfState(state, imuInput, config)
% propagatePoseImuUkfState - 用加速度和角速度传播平台位姿状态
dt = config.sampleTime;
if config.variant == "position_bias_ukf"
    position = state(1:3);
    translationVelocity = state(4:6);
    legLengthBias = state(7:12);
    accelerometerBias = state(13:15);
    orientation = imuInput(8:10);

    acceleration = imuInput(1:3) - accelerometerBias;
    if imuInput(7) > 0.5
        acceleration = rpy2rotmZYX(orientation) * acceleration + config.model.g;
    end

    nextPosition = position + dt * translationVelocity + 0.5 * dt^2 * acceleration;
    nextTranslationVelocity = translationVelocity + dt * acceleration;
    nextState = [nextPosition; nextTranslationVelocity; legLengthBias; accelerometerBias];
    return;
end

if config.variant == "pose_leg_bias_ukf"
    pose = state(1:6);
    velocity = state(7:12);
    legLengthBias = state(13:18);
    accelerometerBias = state(19:21);
    gyroBias = state(22:24);

    acceleration = imuInput(1:3) - accelerometerBias;
    if imuInput(7) > 0.5
        acceleration = rpy2rotmZYX(pose(4:6)) * acceleration + config.model.g;
    end
    angularVelocity = imuInput(4:6) - gyroBias;
    rpyRate = rpyRateMapZYX(pose(4:6)) \ angularVelocity;

    nextPose = pose;
    nextPose(1:3) = pose(1:3) + dt * velocity(1:3) + 0.5 * dt^2 * acceleration;
    nextPose(4:6) = pose(4:6) + dt * rpyRate;
    nextVelocity = [velocity(1:3) + dt * acceleration; rpyRate];
    nextState = [nextPose; nextVelocity; legLengthBias; accelerometerBias; gyroBias];
    return;
end

pose = state(1:6);
velocity = state(7:12);
accelerometerBias = state(13:15);
gyroBias = state(16:18);

acceleration = imuInput(1:3) - accelerometerBias;
if imuInput(7) > 0.5
    acceleration = rpy2rotmZYX(pose(4:6)) * acceleration + config.model.g;
end
angularVelocity = imuInput(4:6) - gyroBias;
rpyRate = rpyRateMapZYX(pose(4:6)) \ angularVelocity;

nextPose = pose;
nextPose(1:3) = pose(1:3) + dt * velocity(1:3) + 0.5 * dt^2 * acceleration;
nextPose(4:6) = pose(4:6) + dt * rpyRate;
nextVelocity = [velocity(1:3) + dt * acceleration; rpyRate];
nextState = [nextPose; nextVelocity; accelerometerBias; gyroBias];
end
