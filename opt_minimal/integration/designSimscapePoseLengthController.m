function design = designSimscapePoseLengthController(model, config)
% designSimscapePoseLengthController - 设计位姿误差到腿长修正的外环映射
arguments
    model struct
    config struct
end

design = designSimscapeLengthCascadeController(config);
design.poseFeedbackGain = config.poseFeedbackGain;
design.poseCorrectionLimit = config.poseCorrectionLimit;
design.poseFeedbackFilterHz = config.poseFeedbackFilterHz;
design.poseFeedbackFilterAlpha = exp(-2 * pi * config.poseFeedbackFilterHz * config.sampleTime);
design.poseFeedbackFilterNumerator = 1 - design.poseFeedbackFilterAlpha;
design.poseFeedbackFilterDenominator = [1, -design.poseFeedbackFilterAlpha];
design.useTimeVaryingReferenceJacobian = true;
homeJacobian = sgpJacobian(model.qHome, model);
design.poseLengthRank = rank(homeJacobian.Jq);
design.stable = design.stable && design.poseLengthRank == 6 && ...
    all(isfinite([design.poseFeedbackFilterNumerator, design.poseFeedbackFilterDenominator]));
end
