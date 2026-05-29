function sqpResult = solveImplicitHSSQP(nlpData, z0, model, scene, disc, optionsConfig)
% solveImplicitHSSQP - 使用 fmincon 求解当前 CasADi 隐式 HS NLP
%
% 文件用途：
%   在不改变 J、gEq、cIneq、lbz、ubz 的前提下，使用 fmincon 求解
%   buildCasadiImplicitHSNLP 已构造的同一个 NLP。默认配置仍为 SQP + CasADi
%   一阶导数 + 变量尺度；当 Algorithm='interior-point' 且 useExactHessian=true 时，
%   额外向 fmincon 提供 CasADi 拉格朗日 Hessian。
%
% 输入：
%   nlpData       - 当前 CasADi NLP 数据
%   z0            - 物理变量初值
%   model/scene/disc - 用于构造尺度和保存上下文
%   optionsConfig - SQP 配置，含 useVariableScaling、useExactGradient、MaxIterations 等
%
% 输出：
%   sqpResult - 物理空间 zOpt、fval、exitflag、output、lambda、grad、hessian、求解时间、
%               迭代历史、尺度信息和最终 NLP 约束残差
%
% 求解链路位置：
%   run_02_compare_ipopt_sqp 调用本函数作为 Solver B。
%
% 是否改变数学问题：
%   否。尺度只改变 fmincon 内部变量；所有函数值和后验验证均使用恢复后的物理 z。

if exist('fmincon', 'file') ~= 2
    error('solveImplicitHSSQP:MissingFmincon', '未找到 fmincon，无法运行 SQP 对比。');
end
if nargin < 6 || isempty(optionsConfig)
    optionsConfig = struct();
end
optionsConfig = fillDefaultOptions(optionsConfig);

if optionsConfig.useVariableScaling
    if isfield(optionsConfig, 'scale') && ~isempty(optionsConfig.scale)
        scale = optionsConfig.scale(:);
        scaleSummary = struct('description', 'scale supplied by optionsConfig', ...
            'minScale', min(scale), 'maxScale', max(scale));
    else
        [scale, scaleSummary] = buildImplicitHSVariableScale(z0, model, scene, disc);
    end
else
    scale = ones(numel(z0), 1);
    scaleSummary = struct('description', 'variable scaling disabled', 'minScale', 1, 'maxScale', 1);
end

scalingConfig = struct();
scalingConfig.useVariableScaling = optionsConfig.useVariableScaling;
scalingConfig.scale = scale;
adapter = buildFminconSQPAdapterFromCasadi(nlpData, scalingConfig);

y0 = adapter.toSolver(z0);
iterationHistory = initializeHistory();
wrappedObjective = adapter.objectiveFcn;
wrappedNonlcon = adapter.nonlconFcn;

fminconOptions = optimoptions('fmincon', ...
    'Algorithm', optionsConfig.Algorithm, ...
    'Display', optionsConfig.Display, ...
    'SpecifyObjectiveGradient', optionsConfig.useExactGradient, ...
    'SpecifyConstraintGradient', optionsConfig.useExactGradient, ...
    'ConstraintTolerance', optionsConfig.ConstraintTolerance, ...
    'OptimalityTolerance', optionsConfig.OptimalityTolerance, ...
    'StepTolerance', optionsConfig.StepTolerance, ...
    'MaxIterations', optionsConfig.MaxIterations, ...
    'MaxFunctionEvaluations', optionsConfig.MaxFunctionEvaluations, ...
    'OutputFcn', @recordIteration);
if strcmpi(optionsConfig.Algorithm, 'interior-point') && optionsConfig.useExactHessian
    fminconOptions = optimoptions(fminconOptions, 'HessianFcn', adapter.hessianFcn);
end

solveTimer = tic;
try
    [yOpt, fval, exitflag, output, lambda, grad, hessian] = fmincon( ...
        wrappedObjective, y0, [], [], [], [], adapter.lby, adapter.uby, wrappedNonlcon, fminconOptions);
    solveError = [];
catch ME
    yOpt = y0;
    fval = NaN;
    exitflag = -999;
    output = struct('message', ME.message, 'iterations', numel(iterationHistory.iteration), 'funcCount', NaN);
    lambda = struct();
    grad = [];
    hessian = [];
    solveError = ME;
end
solveTime = toc(solveTimer);

zOpt = adapter.toPhysical(yOpt);
[Jfinal, gEqFinal, cIneqFinal] = nlpData.eval(zOpt);
Jfinal = full(Jfinal);
gEqFinal = full(gEqFinal);
cIneqFinal = full(cIneqFinal);

sqpResult = struct();
sqpResult.solverName = buildSolverName(optionsConfig);
sqpResult.zOpt = zOpt;
sqpResult.yOpt = yOpt;
sqpResult.fval = fval;
sqpResult.casadiObjectiveAtZOpt = Jfinal;
sqpResult.exitflag = exitflag;
sqpResult.output = output;
sqpResult.lambda = lambda;
sqpResult.grad = grad;
sqpResult.hessian = hessian;
sqpResult.solveTime = solveTime;
sqpResult.iterationHistory = iterationHistory;
sqpResult.functionCount = readOutputField(output, 'funcCount', NaN);
sqpResult.iterations = readOutputField(output, 'iterations', numel(iterationHistory.iteration));
sqpResult.maxEqResidual = max(abs(gEqFinal(:)));
sqpResult.maxIneqViolation = max([cIneqFinal(:); 0]);
sqpResult.gEq = gEqFinal;
sqpResult.cIneq = cIneqFinal;
sqpResult.adapter = rmfield(adapter, {'objectiveFcn', 'nonlconFcn', 'hessianFcn', ...
    'evalFunction', 'hessianFunction', 'toPhysical', 'toSolver'});
sqpResult.scale = scale;
sqpResult.scaleSummary = scaleSummary;
sqpResult.optionsConfig = optionsConfig;
sqpResult.usesExactGradient = optionsConfig.useExactGradient;
sqpResult.usesExactHessian = optionsConfig.useExactHessian;
sqpResult.usesVariableScaling = optionsConfig.useVariableScaling;
sqpResult.success = exitflag > 0 && sqpResult.maxEqResidual <= optionsConfig.ConstraintTolerance && ...
    sqpResult.maxIneqViolation <= optionsConfig.ConstraintTolerance;
sqpResult.solveError = solveError;

    function stop = recordIteration(yCurrent, optimValues, state)
        stop = false;
        if strcmp(state, 'iter') || strcmp(state, 'init') || strcmp(state, 'done')
            zCurrent = adapter.toPhysical(yCurrent);
            [~, gEqCurrent, cIneqCurrent] = nlpData.eval(zCurrent);
            gEqCurrent = full(gEqCurrent);
            cIneqCurrent = full(cIneqCurrent);
            iterationHistory.iteration(end+1, 1) = getOptimValue(optimValues, 'iteration', numel(iterationHistory.iteration)); %#ok<AGROW>
            iterationHistory.fval(end+1, 1) = getOptimValue(optimValues, 'fval', NaN); %#ok<AGROW>
            iterationHistory.maxEqResidual(end+1, 1) = max(abs(gEqCurrent(:))); %#ok<AGROW>
            iterationHistory.maxIneqViolation(end+1, 1) = max([cIneqCurrent(:); 0]); %#ok<AGROW>
            iterationHistory.maxConstraintViolation(end+1, 1) = max([iterationHistory.maxEqResidual(end); iterationHistory.maxIneqViolation(end)]); %#ok<AGROW>
            iterationHistory.firstOrderOpt(end+1, 1) = getOptimValue(optimValues, 'firstorderopt', NaN); %#ok<AGROW>
            iterationHistory.stepSize(end+1, 1) = getOptimValue(optimValues, 'stepsize', NaN); %#ok<AGROW>
            iterationHistory.state{end+1, 1} = state; %#ok<AGROW>
        end
    end
end

function optionsConfig = fillDefaultOptions(optionsConfig)
if ~isfield(optionsConfig, 'useVariableScaling'); optionsConfig.useVariableScaling = true; end
if ~isfield(optionsConfig, 'useExactGradient'); optionsConfig.useExactGradient = true; end
if ~isfield(optionsConfig, 'useExactHessian'); optionsConfig.useExactHessian = false; end
if ~isfield(optionsConfig, 'Algorithm'); optionsConfig.Algorithm = 'sqp'; end
if ~isfield(optionsConfig, 'Display'); optionsConfig.Display = 'iter'; end
if ~isfield(optionsConfig, 'ConstraintTolerance'); optionsConfig.ConstraintTolerance = 1e-6; end
if ~isfield(optionsConfig, 'OptimalityTolerance'); optionsConfig.OptimalityTolerance = 1e-6; end
if ~isfield(optionsConfig, 'StepTolerance'); optionsConfig.StepTolerance = 1e-10; end
if ~isfield(optionsConfig, 'MaxIterations'); optionsConfig.MaxIterations = 300; end
if ~isfield(optionsConfig, 'MaxFunctionEvaluations'); optionsConfig.MaxFunctionEvaluations = 200000; end
end

function solverName = buildSolverName(optionsConfig)
algorithmName = char(optionsConfig.Algorithm);
if strcmpi(algorithmName, 'interior-point')
    solverName = 'fmincon-interior-point exactGradientExactHessianScaled';
elseif ~optionsConfig.useExactGradient
    solverName = 'fmincon-SQP finiteDifference';
elseif ~optionsConfig.useVariableScaling
    solverName = 'fmincon-SQP exactGradientUnscaled';
else
    solverName = 'fmincon-SQP exactGradientScaled';
end
if ~optionsConfig.useVariableScaling
    solverName = strrep(solverName, 'Scaled', 'Unscaled');
end
if strcmpi(algorithmName, 'interior-point') && ~optionsConfig.useExactHessian
    solverName = strrep(solverName, 'ExactHessian', 'BFGSHessian');
end
end

function history = initializeHistory()
history = struct();
history.iteration = [];
history.fval = [];
history.maxEqResidual = [];
history.maxIneqViolation = [];
history.maxConstraintViolation = [];
history.firstOrderOpt = [];
history.stepSize = [];
history.state = {};
end

function value = getOptimValue(optimValues, fieldName, defaultValue)
if isstruct(optimValues) && isfield(optimValues, fieldName)
    value = optimValues.(fieldName);
else
    value = defaultValue;
end
end

function value = readOutputField(output, fieldName, defaultValue)
if isstruct(output) && isfield(output, fieldName)
    value = output.(fieldName);
else
    value = defaultValue;
end
end
