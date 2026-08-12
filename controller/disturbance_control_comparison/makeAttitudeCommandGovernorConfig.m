function config = makeAttitudeCommandGovernorConfig(sampleTime, cutoffHz, ...
        dampingRatio, maxRateDegPerSec, maxAccelerationDegPerSec2)
% makeAttitudeCommandGovernorConfig - Configure the attitude-noise governor.
arguments
    sampleTime (1, 1) double {mustBePositive}
    cutoffHz (1, 1) double {mustBePositive} = 2
    dampingRatio (1, 1) double {mustBePositive} = 1
    maxRateDegPerSec double = [2; 2; 3]
    maxAccelerationDegPerSec2 double = [20; 20; 30]
end

maxRateDegPerSec = expandThree(maxRateDegPerSec, 'maxRateDegPerSec');
maxAccelerationDegPerSec2 = expandThree( ...
    maxAccelerationDegPerSec2, 'maxAccelerationDegPerSec2');
if cutoffHz >= 0.5/sampleTime
    error('makeAttitudeCommandGovernorConfig:CutoffAboveNyquist', ...
        'Command-governor cutoff must be below Nyquist.');
end
config = struct('sampleTime', sampleTime, 'cutoffHz', cutoffHz, ...
    'naturalFrequency', 2*pi*cutoffHz, 'dampingRatio', dampingRatio, ...
    'maxRate', deg2rad(maxRateDegPerSec), ...
    'maxAcceleration', deg2rad(maxAccelerationDegPerSec2));
end

function value = expandThree(value, name)
value = value(:);
if isscalar(value), value = repmat(value, 3, 1); end
validateattributes(value, {'double'}, ...
    {'real', 'finite', 'positive', 'size', [3, 1]}, mfilename, name);
end
