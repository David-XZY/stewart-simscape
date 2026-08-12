function test_80_platform_wrench_waveform
% Window, half-sine, and three scales.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'controller', 'disturbance_control_comparison'));
config = makeDisturbanceComparisonConfig();
cases = buildDisturbanceCaseMatrix(config, 'all');
time = 0:config.sampleTime:7.5;
peaks = zeros(6, numel(config.disturbanceScales));
for index = 1:numel(config.disturbanceScales)
    scale = config.disturbanceScales(index);
    item = cases([cases.disturbanceType] == "wrench" & [cases.scale] == scale);
    wrench = evaluatePlatformWrench(time, item, config);
    assert(all(wrench(:, time < 2.5 | time > 2.9) == 0, 'all'));
    peaks(:, index) = max(abs(wrench), [], 2);
    assert(max(abs(wrench(:, time == 2.7)-scale*config.basePlatformWrench)) < 1e-10);
end
assert(max(abs(peaks(:, 2)-2*peaks(:, 1))) < 1e-10);
assert(max(abs(peaks(:, 3)-3*peaks(:, 1))) < 1e-10);
fprintf('test_80_platform_wrench_waveform passed.\n');
end
