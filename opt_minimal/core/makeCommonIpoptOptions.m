function opts = makeCommonIpoptOptions(solverOptions)
% makeCommonIpoptOptions - standard IHSID 主线的统一 IPOPT 配置
%
% 输入：
%   solverOptions struct - 可选覆盖项，包括 maxIter、printLevel、
%       hessianApproximation、acceptableTol、acceptableIter。
%
% 输出：
%   opts struct - 传给 CasADi nlpsol('ipopt') 的统一配置。
%
% 默认固定使用 MA27 和 limited-memory Hessian；测试可通过输入结构覆盖迭代数和打印级别。
if nargin < 1 || isempty(solverOptions)
    solverOptions = struct();
end

opts = struct();
opts.print_time = true;
opts.ipopt.linear_solver = 'ma27';
opts.ipopt.max_iter = 300;
opts.ipopt.tol = 1e-6;
opts.ipopt.constr_viol_tol = 1e-6;
opts.ipopt.acceptable_tol = 1e-3;
opts.ipopt.acceptable_constr_viol_tol = 1e-6;
opts.ipopt.acceptable_dual_inf_tol = 1e-3;
opts.ipopt.acceptable_compl_inf_tol = 1e-8;
opts.ipopt.acceptable_iter = 1;
opts.ipopt.print_level = 4;
opts.ipopt.hessian_approximation = 'limited-memory';
opts.ipopt.bound_push = 1e-8;
opts.ipopt.bound_frac = 1e-8;
opts.ipopt.mu_strategy = 'adaptive';

if isfield(solverOptions, 'maxIter')
    opts.ipopt.max_iter = solverOptions.maxIter;
end
if isfield(solverOptions, 'printLevel')
    opts.ipopt.print_level = solverOptions.printLevel;
end
if isfield(solverOptions, 'hessianApproximation')
    opts.ipopt.hessian_approximation = char(string(solverOptions.hessianApproximation));
end
if isfield(solverOptions, 'acceptableTol')
    opts.ipopt.acceptable_tol = solverOptions.acceptableTol;
end
if isfield(solverOptions, 'acceptableIter')
    opts.ipopt.acceptable_iter = solverOptions.acceptableIter;
end
if isfield(solverOptions, 'tol')
    opts.ipopt.tol = solverOptions.tol;
end
if isfield(solverOptions, 'constrViolTol')
    opts.ipopt.constr_viol_tol = solverOptions.constrViolTol;
end
if isfield(solverOptions, 'linearSolver')
    opts.ipopt.linear_solver = char(string(solverOptions.linearSolver));
end
end
