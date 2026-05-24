%% run_01_hs_dynamic_opt - 压缩 Hermite-Simpson 预对准轨迹优化入口
% 文件用途：
%   使用 474 个决策变量、240 条压缩 HS 动力学等式和完整路径硬约束，求解 6-UCU
%   Stewart 承载弹体从初始位姿到预对准位姿的自由空间轨迹优化问题。
%
% 输入参数：
%   本脚本无外部输入。模型由 buildOptModelCustom 配置，场景由 buildPreAlignmentScene
%   自动计算 qPre，时间离散固定为 20 区间、21 节点、T=5 s。
%
% 输出参数：
%   在 opt_minimal/results 下保存 compressed 标记的 MAT、命令行日志、诊断文本、PNG 图和
%   5 秒 MP4/GIF 动画。MAT 包含 model、scene、disc、traj、denseReport、solverResult、refs。
%
% 核心公式：
%   z=[Xinternal(:);Fnode(:);Fmid(:)]，Xinternal 为 12x19，Fnode 为 6x21，Fmid 为 6x20。
%   中点状态 Xc=0.5*(Xk+Xk1)+h/8*(fk-fk1)，唯一 HS 等式为
%   Xk1-Xk-h/6*(fk+4*fc+fk1)=0。
%
% 在优化链路中的作用：
%   本脚本是 opt_minimal 默认主入口；Simscape 不参与优化，只在求解后导出 refs。
clear; close all; clc;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(projectRoot, 'src'));
addpath(optRoot);

if exist('fmincon', 'file') ~= 2
    error('run_01_hs_dynamic_opt:MissingFmincon', '需要 Optimization Toolbox：未找到 fmincon。');
end

model = buildOptModelCustom();
scene = buildPreAlignmentScene(model);
disc = buildHSDiscretization(scene);

resultDir = fullfile(optRoot, 'results');
if ~exist(resultDir, 'dir')
    mkdir(resultDir);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
diaryFile = fullfile(resultDir, ['hs_compressed_console_', timestamp, '.txt']);
diary(diaryFile);
diaryCleanup = onCleanup(@() diary('off')); %#ok<NASGU>

fprintf('\n===== compressed HS 文件识别与接口切换 =====\n');
fprintf('默认入口：run_01_hs_dynamic_opt.m\n');
fprintf('新增/调用：packHSDecisionCompressed, unpackHSDecisionCompressed, computeHSMidpointStateCompressed,\n');
fprintf('          buildInitialGuessQuinticHSCompressed, evaluateCompressedTrajectory,\n');
fprintf('          costHSDynamicCompressed, nonlconHSDynamicCompressed。\n');
fprintf('停止主流程调用旧 Q/V/A/Ac 打包、中点状态变量和端点 ceq。\n');

fprintf('\n===== 预对准场景回归检查 =====\n');
fprintf('qPre = [% .8f % .8f % .8f % .8f % .8f % .8f]^T\n', scene.qPre);
fprintf('目标规模：numel(z)=474, numel(ceq)=240, numel(c)=2706。\n');

[z0, initialGuess] = buildInitialGuessQuinticHSCompressed(model, scene, disc);
if numel(z0) ~= 474
    error('run_01_hs_dynamic_opt:InvalidDecisionLength', '初值变量长度必须为 474，当前为 %d。', numel(z0));
end

tic;
J0 = costHSDynamicCompressed(z0, model, scene, disc);
tObj0 = toc;
tic;
[c0, ceq0] = nonlconHSDynamicCompressed(z0, model, scene, disc);
tCon0 = toc;
assert(numel(c0) == 2706, '路径不等式数量应为 2706。');
assert(numel(ceq0) == 240, 'compressed HS 等式数量应为 240。');

initialTraj = rebuildCompressedTrajectory(z0, model, scene, disc);
initialReport = validateTrajectoryDense(initialTraj, model, scene, disc);
initialFile = fullfile(resultDir, ['hs_compressed_initial_', timestamp, '.mat']);
save(initialFile, 'model', 'scene', 'disc', 'z0', 'initialGuess', 'initialTraj', 'initialReport', 'J0', 'c0', 'ceq0');

fprintf('\n===== 五次插值 compressed 初值诊断 =====\n');
fprintf('numel(z0)=%d, numel(c0)=%d, numel(ceq0)=%d\n', numel(z0), numel(c0), numel(ceq0));
fprintf('目标初值 J0=%.8e；目标单次评价 %.3f s；nonlcon 单次评价 %.3f s\n', J0, tObj0, tCon0);
printTrajectorySummary(initialTraj, initialReport, c0, ceq0, model, scene);

[lb, ub] = buildDecisionBoundsCompressed(disc, model);
options = buildFminconOptions();

fprintf('\n===== 开始 fmincon-SQP compressed 完整硬约束 NLP 求解 =====\n');
solveTimer = tic;
[zOpt, fval, exitflag, output, lambda, grad, hessian] = fmincon( ...
    @(z) costHSDynamicCompressed(z, model, scene, disc), ...
    z0, [], [], [], [], lb, ub, ...
    @(z) nonlconHSDynamicCompressed(z, model, scene, disc), options);
solveTime = toc(solveTimer);

traj = rebuildCompressedTrajectory(zOpt, model, scene, disc);
[cOpt, ceqOpt] = nonlconHSDynamicCompressed(zOpt, model, scene, disc);
denseReport = validateTrajectoryDense(traj, model, scene, disc);
solverResult = struct('fval', fval, 'exitflag', exitflag, 'output', output, ...
    'lambda', lambda, 'grad', grad, 'hessian', hessian, 'solveTime', solveTime);
result = analyzeHSResult(traj, denseReport, cOpt, ceqOpt, solverResult);

fprintf('\n===== compressed 最终解诊断 =====\n');
fprintf('求解总耗时 %.3f s，iterations=%d，funcCount=%d\n', ...
    solveTime, output.iterations, output.funcCount);
fprintf('最终目标函数值：%.8e，exitflag=%d\n', fval, exitflag);
fprintf('fmincon 退出信息：%s\n', output.message);
printTrajectorySummary(traj, denseReport, cOpt, ceqOpt, model, scene);
if result.successFlag
    fprintf('最终轨迹通过 compressed HS 节点/中点硬约束验证。\n');
else
    fprintf('最终轨迹未通过全部硬约束验证，请查看 result.constraint 和 denseReport。\n');
end

refs = exportTrajectoryToSimscape(traj, scene);
resultFile = fullfile(resultDir, ['hs_compressed_result_', timestamp, '.mat']);
save(resultFile, 'model', 'scene', 'disc', 'traj', 'denseReport', 'solverResult', 'refs', ...
    'result', 'initialReport', 'J0', 'tObj0', 'tCon0');

plotFiles = plotOptResult(traj, model, scene, result, resultDir, timestamp);
animationFile = animateStewartTrajectory(traj, model, scene, resultDir, timestamp);
logFile = fullfile(resultDir, ['hs_compressed_summary_', timestamp, '.txt']);
writeSummaryLog(logFile, resultFile, plotFiles, animationFile, result, denseReport, solverResult, ...
    initialReport, J0, tObj0, tCon0);

fprintf('结果 MAT 已保存：%s\n', resultFile);
fprintf('诊断文本已保存：%s\n', logFile);
fprintf('MATLAB 命令行日志已保存：%s\n', diaryFile);

function disc = buildHSDiscretization(scene)
% buildHSDiscretization - 构建固定 20 区间 HS 离散参数
disc = struct();
disc.numIntervals = 20;
disc.numNodes = 21;
disc.numMidpoints = 20;
disc.duration = scene.preAlign.duration;
disc.h = disc.duration / disc.numIntervals;
disc.tNode = linspace(0, disc.duration, disc.numNodes);
disc.tMid = disc.tNode(1:end-1) + disc.h/2;
end

function [lb, ub] = buildDecisionBoundsCompressed(disc, model)
% buildDecisionBoundsCompressed - 仅对 Fnode/Fmid 设置 ±2000 N 上下界
internalCount = 12 * (disc.numNodes - 2);
nodeForceCount = 6 * disc.numNodes;
midForceCount = 6 * disc.numMidpoints;
totalLength = internalCount + nodeForceCount + midForceCount;
lb = -inf(totalLength, 1);
ub = inf(totalLength, 1);
forceLower = [repmat(model.actuator.forceMin, disc.numNodes, 1); ...
              repmat(model.actuator.forceMin, disc.numMidpoints, 1)];
forceUpper = [repmat(model.actuator.forceMax, disc.numNodes, 1); ...
              repmat(model.actuator.forceMax, disc.numMidpoints, 1)];
forceStart = internalCount + 1;
lb(forceStart:end) = forceLower;
ub(forceStart:end) = forceUpper;
if totalLength ~= 474
    error('run_01_hs_dynamic_opt:InvalidBoundLength', '压缩变量边界长度必须为 474。');
end
end

function options = buildFminconOptions()
% buildFminconOptions - 构建有限差分 SQP 设置
maxIterations = 1000;
maxIterEnv = str2double(getenv('HS_MAX_ITER'));
if isfinite(maxIterEnv) && maxIterEnv > 0
    maxIterations = maxIterEnv;
    fprintf('检测到 HS_MAX_ITER=%.0f，本次仅按该迭代上限运行诊断。\n', maxIterations);
end
options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'iter-detailed', ...
    'SpecifyObjectiveGradient', false, ...
    'SpecifyConstraintGradient', false, ...
    'FiniteDifferenceType', 'forward', ...
    'ScaleProblem', true, ...
    'MaxIterations', maxIterations, ...
    'MaxFunctionEvaluations', 5e5, ...
    'ConstraintTolerance', 1e-6, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-10);
end

function traj = rebuildCompressedTrajectory(z, model, scene, disc)
% rebuildCompressedTrajectory - 从 compressed 决策变量重建绘图、验证和导出所需轨迹结构
data = evaluateCompressedTrajectory(z, model, scene, disc);
traj = struct();
traj.t = disc.tNode;
traj.tc = disc.tMid;
traj.Xnode = data.Xnode;
traj.Xmid = data.Xmid;
traj.Unode = data.Fnode;
traj.Umid = data.Fmid;
traj.Q = data.Xnode(1:6, :);
traj.V = data.Xnode(7:12, :);
traj.Qmid = data.Xmid(1:6, :);
traj.Vmid = data.Xmid(7:12, :);
traj.xStart = [scene.q0; scene.qd0];
traj.xEnd = [scene.qPre; scene.qdPre];
traj.nodePoints = data.nodePoint;
traj.midPoints = data.midPoint;

traj.L = collectPointField(data.nodePoint, 'L');
traj.Ld = collectPointField(data.nodePoint, 'Ld');
traj.Ldd = collectPointField(data.nodePoint, 'Ldd');
traj.sigmaMin = collectPointScalar(data.nodePoint, 'sigmaMin');
traj.condJ = collectPointScalar(data.nodePoint, 'condJ');
traj.minClearance = collectPointScalar(data.nodePoint, 'minClearance');
traj.Lmid = collectPointField(data.midPoint, 'L');
traj.LdMid = collectPointField(data.midPoint, 'Ld');
traj.LddMid = collectPointField(data.midPoint, 'Ldd');
traj.sigmaMinMid = collectPointScalar(data.midPoint, 'sigmaMin');
traj.condJMid = collectPointScalar(data.midPoint, 'condJ');
traj.minClearanceMid = collectPointScalar(data.midPoint, 'minClearance');
end

function values = collectPointField(points, fieldName)
values = zeros(numel(points{1}.(fieldName)), numel(points));
for index = 1:numel(points)
    values(:, index) = points{index}.(fieldName);
end
end

function values = collectPointScalar(points, fieldName)
values = zeros(1, numel(points));
for index = 1:numel(points)
    values(index) = points{index}.(fieldName);
end
end

function printTrajectorySummary(traj, report, c, ceq, model, scene)
% printTrajectorySummary - 输出 compressed 轨迹约束极值
fprintf('qPre 自动计算值：[% .8f % .8f % .8f % .8f % .8f % .8f]^T\n', scene.qPre);
fprintf('腿长范围 [m]：%.6f 到 %.6f，约束 %.3f 到 %.3f\n', ...
    report.minLength, report.maxLength, model.lmin(1), model.lmax(1));
fprintf('最大 |腿速| [m/s]：%.6f，限制 %.6f\n', report.maxAbsLd, max(model.actuator.ldotMax));
fprintf('最大 |腿加速度| [m/s^2]：%.6f，限制 %.6f\n', report.maxAbsLdd, max(model.actuator.lddotMax));
fprintf('最小 sigmaMin：%.6f，阈值 %.6f；最大 condJ：%.6f，警戒 %.6f\n', ...
    report.minSigmaMin, model.singularity.sigmaMinSafe, report.maxCondJ, model.singularity.condWarning);
fprintf('最小碰撞间隙 [m]：%.6f，安全距离 %.6f\n', report.minClearance, scene.collision.safeDistance);
fprintf('驱动力范围 [N]：%.3f 到 %.3f，限制 %.0f 到 %.0f\n', ...
    min([traj.Unode(:); traj.Umid(:)]), max([traj.Unode(:); traj.Umid(:)]), ...
    min(model.actuator.forceMin), max(model.actuator.forceMax));
fprintf('非线性约束数量：%d，最大正违反量：%.3e\n', numel(c), max([c(:); 0]));
fprintf('compressed HS 等式数量：%d，最大绝对残差：%.3e\n', numel(ceq), max(abs(ceq(:))));
end

function writeSummaryLog(logFile, resultFile, plotFiles, animationFile, result, denseReport, solverResult, initialReport, J0, tObj0, tCon0)
% writeSummaryLog - 保存 UTF-8 文本诊断
fid = fopen(logFile, 'w', 'n', 'UTF-8');
if fid < 0
    warning('run_01_hs_dynamic_opt:LogOpenFailed', '无法写入诊断文本：%s', logFile);
    return;
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'Compressed HS pre-alignment trajectory optimization summary\n');
fprintf(fid, 'MAT: %s\n', resultFile);
fprintf(fid, 'decision variables: 474\n');
fprintf(fid, 'HS equalities: 240\n');
fprintf(fid, 'path inequalities: 2706\n');
fprintf(fid, 'initial objective: %.12e\n', J0);
fprintf(fid, 'initial objective eval time: %.6f s\n', tObj0);
fprintf(fid, 'initial nonlcon eval time: %.6f s\n', tCon0);
fprintf(fid, 'initial max path violation: %.12e\n', initialReport.maxPathViolation);
fprintf(fid, 'initial min clearance: %.12e\n', initialReport.minClearance);
fprintf(fid, 'exitflag: %d\n', solverResult.exitflag);
fprintf(fid, 'iterations: %d\n', solverResult.output.iterations);
fprintf(fid, 'funcCount: %d\n', solverResult.output.funcCount);
fprintf(fid, 'solve time: %.6f s\n', solverResult.solveTime);
fprintf(fid, 'message: %s\n', solverResult.output.message);
fprintf(fid, 'fval: %.12e\n', solverResult.fval);
fprintf(fid, 'final successFlag: %d\n', result.successFlag);
fprintf(fid, 'final max path violation: %.12e\n', result.constraint.maxPathViolation);
fprintf(fid, 'final dense max path violation: %.12e\n', result.constraint.denseMaxPathViolation);
fprintf(fid, 'final HS equality max: %.12e\n', result.err.hsEqualityMax);
fprintf(fid, 'final min clearance: %.12e\n', denseReport.minClearance);
fprintf(fid, 'final max abs force: %.12e\n', denseReport.maxAbsForce);
fprintf(fid, 'plots:\n');
for index = 1:numel(plotFiles)
    fprintf(fid, '  %s\n', plotFiles{index});
end
fprintf(fid, 'animation: %s\n', animationFile);
end
