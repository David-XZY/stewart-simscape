function noise = generateBandLimitedAttitudeNoise(time, standardDeviation, cutoffHz, seed, activeWindow)
% generateBandLimitedAttitudeNoise - Deterministic zero-phase, low-pass noise.
arguments
    time double
    standardDeviation double
    cutoffHz (1, 1) double {mustBePositive}
    seed (1, 1) double {mustBeInteger, mustBeNonnegative}
    activeWindow (1, 2) double
end

time = time(:).';
standardDeviation = standardDeviation(:);
if numel(standardDeviation) ~= 3
    error('generateBandLimitedAttitudeNoise:InvalidStandardDeviation', ...
        'standardDeviation must contain three attitude-axis values.');
end
dt = median(diff(time));
if max(abs(diff(time)-dt)) > 1e-11
    error('generateBandLimitedAttitudeNoise:NonuniformTime', ...
        'The time grid must be uniform.');
end
active = time >= activeWindow(1) & time <= activeWindow(2);
noise = zeros(3, numel(time));
if ~any(active)
    return;
end

previous = rng;
cleanup = onCleanup(@() rng(previous));
rng(seed, 'twister');
white = randn(3, numel(time));
alpha = exp(-2*pi*cutoffHz*dt);
filtered = filter(1-alpha, [1, -alpha], white, [], 2);
filtered = fliplr(filter(1-alpha, [1, -alpha], fliplr(filtered), [], 2));

edgeDuration = min(0.25, 0.25*diff(activeWindow));
taper = ones(1, numel(time));
rise = time >= activeWindow(1) & time < activeWindow(1)+edgeDuration;
fall = time > activeWindow(2)-edgeDuration & time <= activeWindow(2);
taper(rise) = 0.5-0.5*cos(pi*(time(rise)-activeWindow(1))/edgeDuration);
taper(fall) = 0.5-0.5*cos(pi*(activeWindow(2)-time(fall))/edgeDuration);
taper(~active) = 0;
filtered = filtered.*taper;
for axis = 1:3
    values = filtered(axis, active);
    values = values-mean(values);
    scale = sqrt(mean(values.^2));
    if scale <= eps
        error('generateBandLimitedAttitudeNoise:DegenerateNoise', ...
            'The generated noise has zero energy.');
    end
    centered = zeros(1, numel(time));
    centered(active) = values;
    noise(axis, :) = standardDeviation(axis)*centered/scale;
end
clear cleanup;
end
