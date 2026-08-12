function [forceCommand, diagnostic] = solveStrictClfCbfQp( ...
    nominalForce, previousForce, clf, cbf, config)
% solveStrictClfCbfQp - Solve the 7-variable QP with no CBF relaxation.
arguments
    nominalForce double
    previousForce double
    clf struct
    cbf struct
    config struct
end

timer = tic;
nominalForce = nominalForce(:);
previousForce = previousForce(:);
validateattributes(nominalForce, {'double'}, {'real', 'finite', 'size', [6, 1]});
validateattributes(previousForce, {'double'}, {'real', 'finite', 'size', [6, 1]});

forceWeight = diag(config.weights.force);
rateWeight = diag(config.weights.forceRate);
H = blkdiag(forceWeight+rateWeight, config.weights.clfSlack);
linearTerm = [-forceWeight*nominalForce-rateWeight*previousForce; 0];

Aineq = [clf.A, -1; cbf.A, zeros(size(cbf.A, 1), 1)];
bineq = [clf.b; cbf.b];
rateStep = config.forceRateLimit*config.dt;
forceLower = max(config.forceMin, previousForce-rateStep);
forceUpper = min(config.forceMax, previousForce+rateStep);
lowerBound = [forceLower; 0];
upperBound = [forceUpper; inf];

solution = [];
exitflag = -10;
status = "solver_unavailable";
solverOutput = struct();
if all(forceLower <= forceUpper) && exist('quadprog', 'file') == 2
    try
        options = cachedQuadprogOptions(config.solver);
        forceGuess = min(max(nominalForce, forceLower), forceUpper);
        slackGuess = max(clf.A*forceGuess-clf.b, 0);
        initialGuess = [forceGuess; slackGuess];
        [solution, ~, exitflag, solverOutput] = quadprog(H, linearTerm, ...
            Aineq, bineq, [], [], lowerBound, upperBound, initialGuess, options);
        if exitflag > 0
            status = "solved";
        else
            status = "quadprog_failed";
        end
    catch exception
        solverOutput = struct('message', exception.message);
        status = "quadprog_error";
        exitflag = -11;
        solution = [];
    end
elseif any(forceLower > forceUpper)
    status = "inconsistent_force_rate_bounds";
    exitflag = -12;
end

usedFallback = isempty(solution) || exitflag <= 0 || any(~isfinite(solution));
if ~usedFallback
    candidateForce = solution(1:6);
    candidateSlack = solution(7);
    hardViolation = max([cbf.A*candidateForce-cbf.b; ...
        forceLower-candidateForce; candidateForce-forceUpper; 0]);
    if hardViolation > 10*config.solver.constraintTolerance
        usedFallback = true;
        status = "postcheck_failed";
        exitflag = -13;
    end
end

if usedFallback
    if all(forceLower <= forceUpper)
        blended = previousForce + config.fallback.nominalBlend * ...
            (nominalForce-previousForce);
        forceCommand = min(max(blended, forceLower), forceUpper);
    else
        forceCommand = min(max(previousForce, config.forceMin), config.forceMax);
    end
    clfSlack = max(clf.A*forceCommand-clf.b, 0);
else
    forceCommand = candidateForce;
    clfSlack = max(candidateSlack, 0);
end

cbfResidual = cbf.b-cbf.A*forceCommand;
clfResidual = clf.b+clfSlack-clf.A*forceCommand;
hardViolation = max([ -cbfResidual; forceLower-forceCommand; ...
    forceCommand-forceUpper; 0]);
active = cbf.names(cbfResidual <= config.solver.activeTolerance);
boundNames = strings(0, 1);
for index = 1:6
    if forceCommand(index)-forceLower(index) <= config.solver.activeTolerance
        boundNames(end+1, 1) = "force_lower_"+index; %#ok<AGROW>
    end
    if forceUpper(index)-forceCommand(index) <= config.solver.activeTolerance
        boundNames(end+1, 1) = "force_upper_"+index; %#ok<AGROW>
    end
end

diagnostic = struct();
diagnostic.status = status;
diagnostic.exitflag = exitflag;
diagnostic.feasible = ~usedFallback && hardViolation <= ...
    10*config.solver.constraintTolerance;
diagnostic.usedFallback = usedFallback;
diagnostic.fallbackSafetyCertified = false;
diagnostic.clfSlack = clfSlack;
diagnostic.clfResidual = clfResidual;
diagnostic.cbfResidual = cbfResidual;
diagnostic.minimumCbfResidual = min(cbfResidual);
diagnostic.hardConstraintViolation = hardViolation;
diagnostic.activeConstraints = [active; boundNames].';
diagnostic.solveTime = toc(timer);
diagnostic.solverOutput = solverOutput;
diagnostic.hasCbfSlack = false;
diagnostic.decisionVariableCount = 7;
diagnostic.softConstraintCount = 1;
diagnostic.hardConstraintCount = size(cbf.A, 1) + 24;
diagnostic.forceLower = forceLower;
diagnostic.forceUpper = forceUpper;
end

function options = cachedQuadprogOptions(solverConfig)
persistent cachedKey cachedOptions
key = sprintf('%s|%.17g|%.17g|%d', char(solverConfig.algorithm), ...
    solverConfig.constraintTolerance, solverConfig.optimalityTolerance, ...
    solverConfig.maxIterations);
if isempty(cachedKey) || ~strcmp(cachedKey, key)
    cachedOptions = optimoptions('quadprog', 'Display', 'off', ...
        'Algorithm', char(solverConfig.algorithm), ...
        'ConstraintTolerance', solverConfig.constraintTolerance, ...
        'OptimalityTolerance', solverConfig.optimalityTolerance, ...
        'MaxIterations', solverConfig.maxIterations);
    cachedKey = key;
end
options = cachedOptions;
end
