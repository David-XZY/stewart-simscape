function result = solveStewartFgMheWindow(window, initial, config, previous)
% solveStewartFgMheWindow - 求解单个 SC-FG-MHE 滑动窗口
arguments
    window struct
    initial struct
    config struct
    previous struct = struct()
end

[z0, layout] = buildStewartFgMheVariables("pack", initial, config);
result = struct();
result.initial = initial;
result.layout = layout;
result.success = false;
result.exitflag = -999;
result.message = "";
result.fallbackUsed = false;
result.solveTime = 0;

if config.solver.maxIterations <= 0
    result = fillFallback(result, previous, initial, 'solver maxIterations <= 0');
    return;
end

residualFcn = @(z) buildStewartFgMheResiduals(z, layout, window, config, previous);
timerHandle = tic;
try
    if exist('lsqnonlin', 'file') == 2
        options = optimoptions('lsqnonlin', ...
            'Display', config.solver.display, ...
            'MaxIterations', config.solver.maxIterations, ...
            'FunctionTolerance', config.solver.functionTolerance, ...
            'StepTolerance', config.solver.stepTolerance);
        [z, ~, ~, exitflag, output] = lsqnonlin(residualFcn, z0, [], [], options);
        result.exitflag = exitflag;
        if isfield(output, 'message')
            result.message = string(output.message);
        end
    else
        options = optimset('Display', 'off', 'MaxIter', config.solver.maxIterations);
        costFcn = @(z) sum(residualFcn(z).^2);
        [z, ~, exitflag, output] = fminsearch(costFcn, z0, options);
        result.exitflag = exitflag;
        if isfield(output, 'message')
            result.message = string(output.message);
        end
    end
    result.solveTime = toc(timerHandle);
    result.estimate = buildStewartFgMheVariables("unpack", z, config, layout);
    result.success = result.exitflag > 0;
    if ~result.success
        result = fillFallback(result, previous, initial, result.message);
    else
        [result.residual, result.breakdown] = ...
            buildStewartFgMheResiduals(z, layout, window, config, previous);
    end
catch ME
    result.solveTime = toc(timerHandle);
    result.exitflag = -998;
    result.message = string(ME.message);
    result = fillFallback(result, previous, initial, result.message);
end
end

function result = fillFallback(result, previous, initial, message)
result.fallbackUsed = true;
result.success = false;
result.message = string(message);
if isfield(previous, 'estimate')
    result.estimate = previous.estimate;
else
    result.estimate = initial;
end
result.residual = [];
result.breakdown = struct();
end
