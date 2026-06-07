function adapter = buildFminconSQPAdapterFromCasadi(nlpData, scalingConfig)
% buildFminconSQPAdapterFromCasadi - 将当前 CasADi 隐式 HS NLP 适配给 fmincon
%
% 文件用途：
%   直接复用 buildCasadiImplicitHSNLP 返回的 z、J、gEq、cIneq 表达式，生成 fmincon
%   所需的目标函数、非线性约束函数、一阶导数及可选拉格朗日 Hessian。该文件
%   不重新定义目标或约束，只做求解器接口和可选变量尺度变换。
%
% 输入：
%   nlpData       - buildCasadiImplicitHSNLP 输出，含 z/J/gEq/cIneq/lbz/ubz/sizes
%   scalingConfig - struct，含 useVariableScaling 与 scale；z=scale.*y
%
% 输出：
%   adapter - 含 objectiveFcn、nonlconFcn、初值/边界变换、尺度信息和 CasADi 求值函数
%
% 求解链路位置：
%   solveImplicitHSSQP 和 test_03_fmincon_adapter_consistency 调用本函数，确保 SQP
%   与 IPOPT 求解严格相同的 J、gEq、cIneq、lbz、ubz。
%
% 是否改变数学问题：
%   否。启用尺度时只改变 fmincon 内部变量 y，物理 NLP 仍为原始 z 空间问题。

import casadi.*

if nargin < 2 || isempty(scalingConfig)
    scalingConfig = struct();
end
if ~isfield(scalingConfig, 'useVariableScaling')
    scalingConfig.useVariableScaling = false;
end
numVariables = nlpData.sizes.numZ;
if ~isfield(scalingConfig, 'scale') || isempty(scalingConfig.scale)
    scalingConfig.scale = ones(numVariables, 1);
end
scale = scalingConfig.scale(:);
if numel(scale) ~= numVariables || any(~isfinite(scale)) || any(scale <= 0)
    error('buildFminconSQPAdapterFromCasadi:InvalidScale', ...
        'scale 必须为长度 %d 的正有限向量。', numVariables);
end
if ~scalingConfig.useVariableScaling
    scale = ones(numVariables, 1);
end

z = nlpData.z;
J = nlpData.J;
gEq = nlpData.gEq;
cIneq = nlpData.cIneq;
gradJ = gradient(J, z);
JacEq = jacobian(gEq, z);
JacIneq = jacobian(cIneq, z);
lambdaEq = MX.sym('lambdaEq', nlpData.sizes.numEq, 1);
lambdaIneq = MX.sym('lambdaIneq', nlpData.sizes.numIneq, 1);
lagrangian = J + lambdaEq.' * gEq + lambdaIneq.' * cIneq;
hessL = hessian(lagrangian, z);

evalFunction = Function('fmincon_sqp_eval', {z}, ...
    {J, gradJ, gEq, cIneq, JacEq, JacIneq}, ...
    {'z'}, {'J', 'gradJ', 'gEq', 'cIneq', 'JacEq', 'JacIneq'});
hessianFunction = Function('fmincon_lagrangian_hessian', {z, lambdaEq, lambdaIneq}, {hessL}, ...
    {'z', 'lambdaEq', 'lambdaIneq'}, {'hessL'});

adapter = struct();
adapter.objectiveFcn = @objectiveFcn;
adapter.nonlconFcn = @nonlconFcn;
adapter.hessianFcn = @hessianFcn;
adapter.toPhysical = @(y) scale .* y(:);
adapter.toSolver = @(zValue) zValue(:) ./ scale;
adapter.lby = nlpData.lbz(:) ./ scale;
adapter.uby = nlpData.ubz(:) ./ scale;
adapter.scale = scale;
adapter.scalingConfig = scalingConfig;
adapter.evalFunction = evalFunction;
adapter.hessianFunction = hessianFunction;
adapter.sizes = nlpData.sizes;
adapter.usesExactGradient = true;
adapter.usesVariableScaling = scalingConfig.useVariableScaling;

    function [f, gradf] = objectiveFcn(y)
        zValue = scale .* y(:);
        [fCas, gradCas] = evalObjectiveOnly(zValue);
        f = fCas;
        if nargout > 1
            gradf = gradCas .* scale;
        end
    end

    function [c, ceq, GC, GCeq] = nonlconFcn(y)
        zValue = scale .* y(:);
        [~, ~, ceqCas, cCas, jacEqCas, jacIneqCas] = evalAll(zValue);
        c = cCas;
        ceq = ceqCas;
        if nargout > 2
            GC = bsxfun(@times, jacIneqCas.', scale);
            GCeq = bsxfun(@times, jacEqCas.', scale);
        end
    end

    function H = hessianFcn(y, lambda)
        zValue = scale .* y(:);
        lambdaEqValue = zeros(nlpData.sizes.numEq, 1);
        lambdaIneqValue = zeros(nlpData.sizes.numIneq, 1);
        if isstruct(lambda)
            if isfield(lambda, 'eqnonlin') && ~isempty(lambda.eqnonlin)
                lambdaEqValue = lambda.eqnonlin(:);
            end
            if isfield(lambda, 'ineqnonlin') && ~isempty(lambda.ineqnonlin)
                lambdaIneqValue = lambda.ineqnonlin(:);
            end
        end
        Hraw = full(hessianFunction(zValue, lambdaEqValue, lambdaIneqValue));
        Hscaled = bsxfun(@times, bsxfun(@times, Hraw, scale), scale.');
        H = sparse(0.5 * (Hscaled + Hscaled.'));
    end

    function [fValue, gradValue] = evalObjectiveOnly(zValue)
        [fRaw, gradRaw] = evalFunction(zValue);
        fValue = full(fRaw);
        gradValue = full(gradRaw);
        gradValue = gradValue(:);
    end

    function [fValue, gradValue, ceqValue, cValue, jacEqValue, jacIneqValue] = evalAll(zValue)
        [fRaw, gradRaw, ceqRaw, cRaw, jacEqRaw, jacIneqRaw] = evalFunction(zValue);
        fValue = full(fRaw);
        gradValue = full(gradRaw);
        gradValue = gradValue(:);
        ceqValue = full(ceqRaw);
        ceqValue = ceqValue(:);
        cValue = full(cRaw);
        cValue = cValue(:);
        jacEqValue = full(jacEqRaw);
        jacIneqValue = full(jacIneqRaw);
    end
end
