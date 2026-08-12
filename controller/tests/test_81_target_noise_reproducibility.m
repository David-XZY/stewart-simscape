function test_81_target_noise_reproducibility
% Seed, support, RMS, and caller RNG isolation.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'controller', 'disturbance_control_comparison'));
config = makeDisturbanceComparisonConfig();
time = 0:config.sampleTime:7.5;
sigma = deg2rad(config.baseAttitudeNoiseStdDeg);
rng(7); before = rng;
first = generateBandLimitedAttitudeNoise(time, sigma, 2, 101, config.noiseWindow);
after = rng;
second = generateBandLimitedAttitudeNoise(time, sigma, 2, 101, config.noiseWindow);
third = generateBandLimitedAttitudeNoise(time, sigma, 2, 102, config.noiseWindow);
assert(isequal(first, second));
assert(~isequal(first, third));
assert(isequal(before.State, after.State));
active = time >= 1 & time <= 6.5;
assert(all(first(:, ~active) == 0, 'all'));
assert(max(abs(sqrt(mean(first(:, active).^2, 2))-sigma)) < 1e-12);
fprintf('test_81_target_noise_reproducibility passed.\n');
end
