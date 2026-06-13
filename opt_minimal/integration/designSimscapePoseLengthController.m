function design = designSimscapePoseLengthController(model, config)
% designSimscapePoseLengthController - 设计位姿误差到腿长修正的外环映射
arguments
    model struct
    config struct
end

design = designSimscapeLengthCascadeController(config);
jacobian = sgpJacobian(model.qHome, model);
design.poseToLengthGain = config.poseFeedbackGain * jacobian.Jq;
design.poseFeedbackGain = config.poseFeedbackGain;
design.poseCorrectionLimit = config.poseCorrectionLimit;
design.poseLengthRank = rank(design.poseToLengthGain);
design.stable = design.stable && design.poseLengthRank == 6 && ...
    all(isfinite(design.poseToLengthGain), 'all');
end
