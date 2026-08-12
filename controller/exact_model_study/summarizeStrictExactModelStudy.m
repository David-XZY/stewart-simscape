function summary = summarizeStrictExactModelStudy(runs)
% summarizeStrictExactModelStudy - Flatten publication metrics to one CSV table.
runCount = numel(runs);
scenarioId = strings(runCount, 1);
displayName = strings(runCount, 1);
studyGroup = strings(runCount, 1);
controllerMode = strings(runCount, 1);
disturbance = strings(runCount, 1);
guaranteeApplicable = false(runCount, 1);
translationRmseM = zeros(runCount, 1);
translationPeakM = zeros(runCount, 1);
rotationRmseRad = zeros(runCount, 1);
rotationPeakRad = zeros(runCount, 1);
legLengthMinM = zeros(runCount, 1);
legLengthMaxM = zeros(runCount, 1);
legLengthLowerMarginMinM = zeros(runCount, 1);
legLengthUpperMarginMinM = zeros(runCount, 1);
legSpeedPeakMps = zeros(runCount, 1);
legAccelerationPeakMps2 = zeros(runCount, 1);
collisionDistanceMinM = zeros(runCount, 1);
collisionMarginMinM = zeros(runCount, 1);
roofDistanceMinM = zeros(runCount, 1);
leftDistanceMinM = zeros(runCount, 1);
rightDistanceMinM = zeros(runCount, 1);
certificateMarginMinM = zeros(runCount, 1);
sigmaMin = zeros(runCount, 1);
sigmaLowerMin = zeros(runCount, 1);
sigmaLowerBoundExcessMax = zeros(runCount, 1);
collisionCertificateExcessMaxM = zeros(runCount, 1);
commandForcePeakN = zeros(runCount, 1);
appliedForcePeakN = zeros(runCount, 1);
commandForceRatePeakNps = zeros(runCount, 1);
appliedForceRatePeakNps = zeros(runCount, 1);
controlEnergyN2s = zeros(runCount, 1);
clfSlackMax = nan(runCount, 1);
clfSlackMean = nan(runCount, 1);
cbfCommandResidualMin = zeros(runCount, 1);
cbfRealizedResidualMin = zeros(runCount, 1);
cbfStateMarginMin = zeros(runCount, 1);
hocbfPsi1Min = zeros(runCount, 1);
infeasibleCount = zeros(runCount, 1);
fallbackCount = zeros(runCount, 1);
activeSampleFraction = zeros(runCount, 1);
mostActiveConstraint = strings(runCount, 1);
qpMeanMs = nan(runCount, 1);
qpP95Ms = nan(runCount, 1);
qpMaxMs = nan(runCount, 1);
trackingPassed = false(runCount, 1);
physicalLimitsPassed = false(runCount, 1);
engineeringPassed = false(runCount, 1);
strictMatchedPassed = false(runCount, 1);
terminatedEarly = false(runCount, 1);
terminationTimeS = nan(runCount, 1);
terminationReason = strings(runCount, 1);
lengthViolationCount = zeros(runCount, 1);
speedViolationCount = zeros(runCount, 1);
accelerationViolationCount = zeros(runCount, 1);
collisionViolationCount = zeros(runCount, 1);
singularityViolationCount = zeros(runCount, 1);
forceViolationCount = zeros(runCount, 1);
forceRateViolationCount = zeros(runCount, 1);
commandCbfViolationCount = zeros(runCount, 1);
realizedCbfViolationCount = zeros(runCount, 1);

for index = 1:runCount
    scenario = runs(index).scenario;
    metrics = runs(index).metrics;
    scenarioId(index) = scenario.id;
    displayName(index) = scenario.displayName;
    studyGroup(index) = scenario.studyGroup;
    controllerMode(index) = scenario.controllerMode;
    disturbance(index) = scenario.disturbance;
    guaranteeApplicable(index) = scenario.guaranteeApplicable;
    translationRmseM(index) = metrics.translationRmse;
    translationPeakM(index) = metrics.translationPeak;
    rotationRmseRad(index) = metrics.rotationRmse;
    rotationPeakRad(index) = metrics.rotationPeak;
    legLengthMinM(index) = metrics.minLegLength;
    legLengthMaxM(index) = metrics.maxLegLength;
    legLengthLowerMarginMinM(index) = metrics.minLegLengthMargin;
    legLengthUpperMarginMinM(index) = metrics.maxLegLengthMargin;
    legSpeedPeakMps(index) = metrics.maxLegSpeed;
    legAccelerationPeakMps2(index) = metrics.maxLegAcceleration;
    collisionDistanceMinM(index) = metrics.minActualCollisionDistance;
    collisionMarginMinM(index) = metrics.minActualCollisionMargin;
    roofDistanceMinM(index) = metrics.minRoofDistance;
    leftDistanceMinM(index) = metrics.minLeftDistance;
    rightDistanceMinM(index) = metrics.minRightDistance;
    certificateMarginMinM(index) = metrics.minCollisionCertificateMargin;
    sigmaMin(index) = metrics.minSigma;
    sigmaLowerMin(index) = metrics.minSigmaLower;
    sigmaLowerBoundExcessMax(index) = metrics.maxSigmaLowerBoundExcess;
    collisionCertificateExcessMaxM(index) = ...
        metrics.maxCollisionCertificateExcess;
    commandForcePeakN(index) = metrics.peakCommandForce;
    appliedForcePeakN(index) = metrics.peakAppliedForce;
    commandForceRatePeakNps(index) = metrics.peakCommandForceRate;
    appliedForceRatePeakNps(index) = metrics.peakAppliedForceRate;
    controlEnergyN2s(index) = metrics.controlEnergy;
    clfSlackMax(index) = metrics.maxClfSlack;
    clfSlackMean(index) = metrics.meanClfSlack;
    cbfCommandResidualMin(index) = metrics.minCommandCbfResidual;
    cbfRealizedResidualMin(index) = metrics.minRealizedCbfResidual;
    cbfStateMarginMin(index) = metrics.minStateMargin;
    hocbfPsi1Min(index) = metrics.minPsi1;
    infeasibleCount(index) = metrics.infeasibleCount;
    fallbackCount(index) = metrics.fallbackCount;
    activeSampleFraction(index) = metrics.activeSampleFraction;
    mostActiveConstraint(index) = metrics.mostActiveConstraint;
    qpMeanMs(index) = 1e3*metrics.qpTimeMean;
    qpP95Ms(index) = 1e3*metrics.qpTimeP95;
    qpMaxMs(index) = 1e3*metrics.qpTimeMax;
    trackingPassed(index) = metrics.trackingPassed;
    physicalLimitsPassed(index) = metrics.physicalLimitsPassed;
    engineeringPassed(index) = metrics.engineeringPassed;
    strictMatchedPassed(index) = metrics.strictMatchedPassed;
    terminatedEarly(index) = metrics.terminatedEarly;
    terminationTimeS(index) = metrics.terminationTime;
    terminationReason(index) = runs(index).termination.reason;
    lengthViolationCount(index) = metrics.lengthViolationCount;
    speedViolationCount(index) = metrics.speedViolationCount;
    accelerationViolationCount(index) = metrics.accelerationViolationCount;
    collisionViolationCount(index) = metrics.collisionViolationCount;
    singularityViolationCount(index) = metrics.singularityViolationCount;
    forceViolationCount(index) = metrics.forceViolationCount;
    forceRateViolationCount(index) = metrics.forceRateViolationCount;
    commandCbfViolationCount(index) = metrics.commandCbfViolationCount;
    realizedCbfViolationCount(index) = metrics.realizedCbfViolationCount;
end

summary = table(scenarioId, displayName, studyGroup, controllerMode, ...
    disturbance, guaranteeApplicable, translationRmseM, translationPeakM, ...
    rotationRmseRad, rotationPeakRad, legLengthMinM, legLengthMaxM, ...
    legLengthLowerMarginMinM, legLengthUpperMarginMinM, legSpeedPeakMps, ...
    legAccelerationPeakMps2, collisionDistanceMinM, collisionMarginMinM, ...
    roofDistanceMinM, leftDistanceMinM, rightDistanceMinM, ...
    certificateMarginMinM, sigmaMin, sigmaLowerMin, ...
    sigmaLowerBoundExcessMax, collisionCertificateExcessMaxM, ...
    commandForcePeakN, ...
    appliedForcePeakN, commandForceRatePeakNps, appliedForceRatePeakNps, ...
    controlEnergyN2s, clfSlackMax, clfSlackMean, cbfCommandResidualMin, ...
    cbfRealizedResidualMin, cbfStateMarginMin, hocbfPsi1Min, ...
    infeasibleCount, fallbackCount, activeSampleFraction, ...
    mostActiveConstraint, qpMeanMs, qpP95Ms, qpMaxMs, trackingPassed, ...
    physicalLimitsPassed, engineeringPassed, strictMatchedPassed, ...
    terminatedEarly, terminationTimeS, terminationReason, ...
    lengthViolationCount, speedViolationCount, accelerationViolationCount, ...
    collisionViolationCount, singularityViolationCount, forceViolationCount, ...
    forceRateViolationCount, commandCbfViolationCount, ...
    realizedCbfViolationCount);
end
