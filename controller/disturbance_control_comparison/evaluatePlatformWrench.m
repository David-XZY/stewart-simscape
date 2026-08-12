function wrench = evaluatePlatformWrench(time, experimentCase, config)
% evaluatePlatformWrench - Six-dimensional half-sine platform disturbance.
arguments
    time double
    experimentCase struct
    config struct
end

wrench = zeros(6, numel(time));
if ~experimentCase.hasWrench
    if isscalar(time), wrench = wrench(:, 1); end
    return;
end
t0 = config.wrenchWindow(1);
t1 = config.wrenchWindow(2);
phase = (time(:).'-t0)/(t1-t0);
active = phase >= 0 & phase <= 1;
wrench(:, active) = experimentCase.scale*config.basePlatformWrench(:) .* ...
    sin(pi*phase(active));
if isscalar(time), wrench = wrench(:, 1); end
end
