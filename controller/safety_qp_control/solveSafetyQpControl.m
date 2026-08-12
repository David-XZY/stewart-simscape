function [Fcmd, diagnostic] = solveSafetyQpControl(Fnom, Fprev, clf, cbf, config)
% solveSafetyQpControl - 求解单步 CLF-CBF-QP 并提供可解释诊断
arguments
    Fnom double
    Fprev double
    clf struct
    cbf struct
    config struct
end

ticHandle = tic;
Fnom = Fnom(:);
Fprev = Fprev(:);
cbfCount = numel(cbf.b);
variableCount = 6 + 1 + cbfCount;

H = zeros(variableCount);
H(1:6, 1:6) = diag(config.weights.force) + config.weights.forceRate * eye(6);
H(7, 7) = config.weights.clfSlack;
if cbfCount > 0
    H(8:end, 8:end) = config.weights.cbfSlack * eye(cbfCount);
end
H = 2 * H;
f = zeros(variableCount, 1);
f(1:6) = -2 * (diag(config.weights.force) * Fnom + config.weights.forceRate * Fprev);

Aineq = [];
bineq = [];
if config.enableClf
    Aineq = [Aineq; clf.A, -1, zeros(1, cbfCount)];
    bineq = [bineq; clf.b];
end
if config.enableCbf && cbfCount > 0
    Aineq = [Aineq; cbf.A, zeros(cbfCount, 1), -eye(cbfCount)];
    bineq = [bineq; cbf.b];
end

lowerBound = [-inf(6, 1); 0; zeros(cbfCount, 1)];
upperBound = [inf(6, 1); inf; inf(cbfCount, 1)];
if config.forceInfeasibleForTest
    lowerBound(1:6) = config.forceMax + 1;
    upperBound(1:6) = config.forceMin - 1;
end

[solution, exitflag, statusText] = runQp(H, f, Aineq, bineq, lowerBound, upperBound);
usedFallback = isempty(solution) || exitflag <= 0 || any(~isfinite(solution(1:6)));
if usedFallback
    Fcmd = fallbackSafetyForce(Fnom, Fprev, config);
    exitflag = min(exitflag, -1);
    statusText = "fallback";
    clfSlack = max(clf.A * Fcmd - clf.b, 0);
    cbfSlack = max(cbf.A * Fcmd - cbf.b, 0);
else
    Fcmd = solution(1:6);
    clfSlack = solution(7);
    cbfSlack = solution(8:end);
end
Fcmd = enforceForceLimits(Fcmd, Fprev, config);

constraintValue = cbf.A * Fcmd - cbf.b;
activeConstraints = cbf.names(constraintValue >= -1e-6);
diagnostic = struct();
diagnostic.status = statusText;
diagnostic.exitflag = exitflag;
diagnostic.activeConstraints = activeConstraints(:).';
diagnostic.clfViolation = max(clf.A * Fcmd - clf.b, 0);
diagnostic.cbfViolation = max([constraintValue; 0]);
diagnostic.clfSlack = clfSlack;
diagnostic.cbfSlack = cbfSlack;
diagnostic.solveTime = toc(ticHandle);
diagnostic.safetyMargin = cbf.minMargin;
diagnostic.usedFallback = usedFallback;
diagnostic.safetyBrakeCount = double(usedFallback);
diagnostic.cbf = cbf;
diagnostic.clf = clf;
end

function [solution, exitflag, statusText] = runQp(H, f, Aineq, bineq, lowerBound, upperBound)
solution = [];
exitflag = -10;
statusText = "unavailable";
if exist('quadprog', 'file') == 2
    try
        options = optimoptions('quadprog', 'Display', 'off');
        [solution, ~, exitflag] = quadprog(H, f, Aineq, bineq, [], [], ...
            lowerBound, upperBound, [], options);
        if exitflag > 0
            statusText = "solved";
        else
            statusText = "quadprog_failed";
        end
        return;
    catch
        solution = [];
        exitflag = -11;
        statusText = "quadprog_error";
    end
end
end

function Fsafe = fallbackSafetyForce(Fnom, Fprev, config)
Fsafe = Fprev + config.fallback.nominalScale * (Fnom - Fprev);
Fsafe = Fsafe - config.fallback.safeBrakeDamping * config.dt * sign(Fsafe);
end

function Fcmd = enforceForceLimits(Fcmd, Fprev, config)
rateStep = config.forceRateLimit(:) * config.dt;
Fcmd = min(max(Fcmd, Fprev - rateStep), Fprev + rateStep);
Fcmd = min(max(Fcmd, config.forceMin), config.forceMax);
end
