function test_89_attitude_command_governor_contract
% Governor must be deterministic, bounded, smooth, and zero-input preserving.
controllerRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(controllerRoot, 'disturbance_control_comparison'));
config = makeAttitudeCommandGovernorConfig(0.01, 2, 1, [2; 2; 3], ...
    [20; 20; 30]);
state = initializeAttitudeCommandGovernor();
stateReplay = initializeAttitudeCommandGovernor();
raw = deg2rad([0.5; -0.4; 0.3]);
previousPosition = state.perturbation;
for index = 1:300
    [command, state] = stepAttitudeCommandGovernor(raw, state, config);
    [replay, stateReplay] = stepAttitudeCommandGovernor(raw, stateReplay, config);
    assert(max(abs(command.perturbation-replay.perturbation)) < 1e-15);
    assert(max(abs(command.rate-replay.rate)) < 1e-15);
    assert(all(abs(command.rate) <= config.maxRate+1e-12));
    assert(all(abs(command.acceleration) <= config.maxAcceleration+1e-12));
    assert(max(abs((command.perturbation-previousPosition)/config.sampleTime- ...
        command.rate)) < 1e-12);
    previousPosition = command.perturbation;
end
assert(max(abs(command.perturbation-raw)) < deg2rad(1e-3));

zeroState = initializeAttitudeCommandGovernor();
[zeroCommand, zeroState] = stepAttitudeCommandGovernor( ...
    zeros(3, 1), zeroState, config);
assert(max(abs([zeroCommand.perturbation; zeroCommand.rate; ...
    zeroCommand.acceleration; zeroState.perturbation; zeroState.rate])) == 0);
fprintf('test_89_attitude_command_governor_contract passed.\n');
end
