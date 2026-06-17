function comparison = comparePoseTrackingPerformance(candidate, baseline, characteristicLength)
% comparePoseTrackingPerformance - 按等效位姿误差比较 Run02 与 Run03
arguments
    candidate struct
    baseline struct
    characteristicLength (1,1) double {mustBePositive}
end

candidateEquivalent = equivalentPoseError(candidate.poseError, characteristicLength);
baselineEquivalent = equivalentPoseError(baseline.poseError, characteristicLength);
candidateRms = sqrt(mean(candidateEquivalent.^2, 'all'));
baselineRms = sqrt(mean(baselineEquivalent.^2, 'all'));
candidatePeak = max(abs(candidateEquivalent), [], 'all');
baselinePeak = max(abs(baselineEquivalent), [], 'all');

comparison = struct();
comparison.characteristicLength = characteristicLength;
comparison.candidateEquivalentPoseRms = candidateRms;
comparison.baselineEquivalentPoseRms = baselineRms;
comparison.candidateEquivalentPosePeak = candidatePeak;
comparison.baselineEquivalentPosePeak = baselinePeak;
comparison.rmsRatio = candidateRms / baselineRms;
comparison.peakRatio = candidatePeak / baselinePeak;
comparison.score = 0.5 * (comparison.rmsRatio + comparison.peakRatio);
comparison.translationPeakRatio = candidate.metrics.maxTranslationPeak / ...
    baseline.metrics.maxTranslationPeak;
comparison.rotationPeakRatio = candidate.metrics.maxRotationPeak / ...
    baseline.metrics.maxRotationPeak;
comparison.passed = candidate.passed && comparison.score <= 1 && ...
    comparison.translationPeakRatio <= 1.1 && comparison.rotationPeakRatio <= 1.1;
end

function value = equivalentPoseError(poseError, characteristicLength)
value = [poseError(:, 1:3), characteristicLength * poseError(:, 4:6)];
end
