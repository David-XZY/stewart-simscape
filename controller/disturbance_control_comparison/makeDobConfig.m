function config = makeDobConfig(sampleTime, cutoffHz, legForceLimit, rateLimit)
% makeDobConfig - Configure the six-dimensional generalized-wrench DOB.
arguments
    sampleTime (1, 1) double {mustBePositive}
    cutoffHz (1, 1) double {mustBePositive}
    legForceLimit (1, 1) double {mustBePositive}
    rateLimit (1, 1) double {mustBePositive}
end

if cutoffHz >= 0.5/sampleTime
    error('makeDobConfig:CutoffAboveNyquist', ...
        'DOB cutoff must be below Nyquist.');
end
config = struct();
config.sampleTime = sampleTime;
config.cutoffHz = cutoffHz;
config.filterAlpha = exp(-2*pi*cutoffHz*sampleTime);
config.legForceLimit = repmat(legForceLimit, 6, 1);
config.rateLimit = repmat(rateLimit, 6, 1);
config.mappingRcondMin = 1e-10;
end
