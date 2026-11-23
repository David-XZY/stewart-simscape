function state = initializeDisturbanceObserver()
% initializeDisturbanceObserver - Zero-bias initial DOB state.
state = struct('wrenchEstimate', zeros(6, 1), ...
    'legCompensation', zeros(6, 1), 'initialized', false);
end
