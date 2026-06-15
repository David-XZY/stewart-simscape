function nextState = propagatePoseImuUkfState(state, imuInput, config)
% propagatePoseImuUkfState - 用加速度和角速度传播平台位姿状态
dt = config.sampleTime;
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
