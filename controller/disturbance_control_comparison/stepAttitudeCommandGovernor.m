function [command, state, diagnostic] = stepAttitudeCommandGovernor( ...
        rawPerturbation, state, config)
% stepAttitudeCommandGovernor - Rate/acceleration-limited second-order filter.
arguments
    rawPerturbation double
    state struct
    config struct
end

rawPerturbation = reshape(rawPerturbation, 3, 1);
position = reshape(state.perturbation, 3, 1);
rate = reshape(state.rate, 3, 1);
validateattributes(rawPerturbation, {'double'}, ...
    {'real', 'finite', 'size', [3, 1]});
omega = config.naturalFrequency;
rawAcceleration = omega^2*(rawPerturbation-position)- ...
    2*config.dampingRatio*omega*rate;
acceleration = min(max(rawAcceleration, -config.maxAcceleration), ...
    config.maxAcceleration);
nextRate = rate+config.sampleTime*acceleration;
nextRate = min(max(nextRate, -config.maxRate), config.maxRate);
acceleration = (nextRate-rate)/config.sampleTime;
nextPosition = position+config.sampleTime*nextRate;

state.perturbation = nextPosition;
state.rate = nextRate;
command = struct('perturbation', nextPosition, 'rate', nextRate, ...
    'acceleration', acceleration);
diagnostic = struct('rawPerturbation', rawPerturbation, ...
    'rawAcceleration', rawAcceleration, ...
    'rateLimitActive', any(abs(nextRate) >= config.maxRate-10*eps), ...
    'accelerationLimitActive', any(abs(rawAcceleration) > ...
    config.maxAcceleration+eps));
end
