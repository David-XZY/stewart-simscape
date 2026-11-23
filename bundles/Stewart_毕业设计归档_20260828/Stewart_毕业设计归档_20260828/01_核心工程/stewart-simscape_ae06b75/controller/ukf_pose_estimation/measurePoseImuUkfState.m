function measurement = measurePoseImuUkfState(state, config, orientation)
% measurePoseImuUkfState - 预测相对腿长和三轴姿态测量
if config.variant == "position_bias_ukf"
    pose = [state(1:3); orientation(:)];
    relativeLength = sgpIK(pose, config.model).L - config.anchorLength + state(7:12);
    measurement = relativeLength;
    return;
end

pose = state(1:6);
relativeLength = sgpIK(pose, config.model).L - config.anchorLength;
if config.variant == "pose_leg_bias_ukf"
    relativeLength = relativeLength + state(13:18);
end
measurement = [relativeLength; pose(4:6)];
end
