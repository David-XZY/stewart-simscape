function measurement = measurePoseImuUkfState(state, config)
% measurePoseImuUkfState - 预测相对腿长和三轴姿态测量
pose = state(1:6);
relativeLength = sgpIK(pose, config.model).L - config.anchorLength;
measurement = [relativeLength; pose(4:6)];
end
