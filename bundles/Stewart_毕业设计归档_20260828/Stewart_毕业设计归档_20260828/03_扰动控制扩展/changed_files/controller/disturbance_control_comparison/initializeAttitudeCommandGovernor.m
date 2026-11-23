function state = initializeAttitudeCommandGovernor(initialPerturbation, initialRate)
% initializeAttitudeCommandGovernor - Create the three-axis governor state.
arguments
    initialPerturbation double = zeros(3, 1)
    initialRate double = zeros(3, 1)
end
state = struct('perturbation', reshape(initialPerturbation, 3, 1), ...
    'rate', reshape(initialRate, 3, 1));
validateattributes(state.perturbation, {'double'}, ...
    {'real', 'finite', 'size', [3, 1]});
validateattributes(state.rate, {'double'}, ...
    {'real', 'finite', 'size', [3, 1]});
end
