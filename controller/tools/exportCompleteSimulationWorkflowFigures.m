function manifest = exportCompleteSimulationWorkflowFigures(outputDir)
% exportCompleteSimulationWorkflowFigures - 导出完整仿真流程技术报告配图
%
% 说明：
%   本工具只读取现有模型和权威结果文件，不修改控制器、参数或 SLX。
%   数据图同时导出 PNG/PDF；Simulink 层级图优先导出真实模型截图，
%   若当前 MATLAB 环境不支持模型截图，则使用清晰的层级示意图替代。
arguments
    outputDir {mustBeTextScalar} = ""
end

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
optRoot = fullfile(projectRoot, 'opt_minimal');
controllerRoot = fullfile(projectRoot, 'controller');
resultRoot = fullfile(projectRoot, 'results', 'controller');
if strlength(string(outputDir)) == 0
    outputDir = fullfile(projectRoot, 'docs', 'controller_workflow', 'figures', 'simulation_workflow');
end
outputDir = char(outputDir);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'matlab'));
addpath(fullfile(projectRoot, 'matlab', 'simscape_subsystems'));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));
addpath(fullfile(controllerRoot, 'pwm_identification'), fullfile(controllerRoot, 'ukf_pose_estimation'));

files = resolveFiles(resultRoot, optRoot);
data = loadAuthorityData(files);
model = buildOptModelCustom();
teacher = data.identifier.teacher;

records = strings(0, 3);
records(end + 1, :) = saveFigure(makeArchitectureFigure(), outputDir, "01_project_architecture");
records(end + 1, :) = saveFigure(makeGeometryFigure(model), outputDir, "02_stewart_geometry");
records(end + 1, :) = saveFigure(makeAnchorMappingFigure(model), outputDir, "03_anchor_mapping");
records(end + 1, :) = saveFigure(makeSingleLegFigure(model), outputDir, "04_single_leg_vector");
records(end + 1, :) = saveFigure(makeKinematicsFlowFigure(), outputDir, "05_kinematics_dynamics_flow");
records(end + 1, :) = saveFigure(makeTrajectoryKinematicsFigure(data.refs, model), outputDir, "06_trajectory_kinematics");

records(end + 1, :) = exportOrDrawSystem(projectRoot, outputDir, ...
    "stewart_platform_model", "07_simscape_top_level", @makeSimscapeTopFigure);
records(end + 1, :) = saveFigure(makeSimscapePlatformFigure(), outputDir, "08_simscape_platform_layer");
records(end + 1, :) = exportOrDrawSystem(projectRoot, outputDir, ...
    "stewart_strut", "09_simscape_strut_layer", @makeStrutFigure);
records(end + 1, :) = saveFigure(makeParameterMappingFigure(), outputDir, "10_parameter_mapping");

records(end + 1, :) = saveFigure(makeReferenceReconstructionFigure(data.refs), outputDir, "11_reference_reconstruction");
records(end + 1, :) = saveFigure(makeReferenceDataflowFigure(), outputDir, "12_reference_dataflow");
records(end + 1, :) = saveFigure(makeRun02ControlFigure(), outputDir, "13_run02_control_structure");
records(end + 1, :) = saveFigure(makeRun02DesignFigure(), outputDir, "14_run02_controller_design");
records(end + 1, :) = saveFigure(makeRun02ResultFigure(data.run02.report), outputDir, "15_run02_tracking_results");
records(end + 1, :) = saveFigure(makeControlComparisonFigure(), outputDir, "16_run02_run03_run04_structures");
records(end + 1, :) = saveFigure(makeThreeRunResultFigure(data), outputDir, "17_run02_run03_run04_results");

records(end + 1, :) = saveFigure(makePwmChainFigure(), outputDir, "18_pwm_physical_chain");
records(end + 1, :) = saveFigure(makePwmCharacteristicFigure(teacher), outputDir, "19_pwm_force_characteristics");
records(end + 1, :) = saveFigure(makeFrictionFigure(teacher), outputDir, "20_friction_force_decomposition");
records(end + 1, :) = saveFigure(makeSwitchingFigure(teacher), outputDir, "21_average_switching_validation");

records(end + 1, :) = saveFigure(makeIdentificationFlowFigure(), outputDir, "22_identification_pipeline");
records(end + 1, :) = saveFigure(makeCoverageFigure(data.identifier.dataset), outputDir, "23_identification_coverage");
records(end + 1, :) = saveFigure(makeGrayParameterFigure(data.identifier), outputDir, "24_gray_parameter_fit");
records(end + 1, :) = saveFigure(makeIdentificationResultFigure(data.identifier), outputDir, "25_gray_narx_validation");
records(end + 1, :) = saveFigure(makeIdentificationMetricFigure(data.identifier), outputDir, "26_identification_metrics");

records(end + 1, :) = saveFigure(makeUkfFigure(), outputDir, "27_ukf_estimation_flow");
records(end + 1, :) = saveFigure(makePoseBenchmarkFigure(data), outputDir, "28_pose_estimator_benchmark");
records(end + 1, :) = saveFigure(makeForceAlignmentFigure(data.comparison), outputDir, "29_force_estimate_alignment");
records(end + 1, :) = saveFigure(makePwmClosedLoopFigure(), outputDir, "30_pwm_closed_loop");
records(end + 1, :) = saveFigure(makeForcePwmTimelineFigure(data.comparison), outputDir, "31_force_pwm_timeline");
records(end + 1, :) = saveFigure(makeUtilizationFigure(data), outputDir, "32_control_utilization");
records(end + 1, :) = saveFigure(makeExecutionFlowFigure(), outputDir, "33_execution_validation_flow");
records(end + 1, :) = saveFigure(makeTestMatrixFigure(), outputDir, "34_test_matrix");
records(end + 1, :) = saveFigure(makeMacroComparisonFigure(data.macro), outputDir, "35_macro_motion_comparison");
records(end + 1, :) = saveFigure(makeErrorSourceFigure(data.macro), outputDir, "36_error_source_priorities");

manifest = table(records(:, 1), records(:, 2), records(:, 3), ...
    'VariableNames', {'name', 'png', 'pdf'});
writetable(manifest, fullfile(outputDir, 'manifest.csv'));
save(fullfile(outputDir, 'manifest.mat'), 'manifest', 'files');
fprintf('完整仿真流程报告配图已导出：%s\n', outputDir);
end

function files = resolveFiles(resultRoot, optRoot)
files = struct();
files.run02 = latestFile(resultRoot, 'result_simscape_pose_force_control_*.mat');
files.run03 = latestFile(resultRoot, 'result_simscape_length_cascade_*.mat');
files.run04 = latestFile(resultRoot, 'result_simscape_pose_length_control_*.mat');
files.identifier = latestFile(resultRoot, 'pwm_force_identifier_*.mat');
files.comparison = latestFile(resultRoot, 'pwm_pose_force_comparison_*.mat');
files.macro = latestFile(resultRoot, 'ideal_pwm_macro_comparison_*', true);
files.refs = fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', 'simscape_references.mat');
end

function fileName = latestFile(folder, pattern, folderMode)
if nargin < 3
    folderMode = false;
end
items = dir(fullfile(folder, pattern));
if folderMode
    items = items([items.isdir]);
end
if isempty(items)
    error('exportCompleteSimulationWorkflowFigures:MissingInput', ...
        '未找到权威输入：%s', fullfile(folder, pattern));
end
[~, index] = max([items.datenum]);
if folderMode
    fileName = fullfile(items(index).folder, items(index).name, 'comparison.mat');
else
    fileName = fullfile(items(index).folder, items(index).name);
end
end

function data = loadAuthorityData(files)
data = struct();
data.run02 = load(files.run02);
data.run03 = load(files.run03);
data.run04 = load(files.run04);
data.identifier = load(files.identifier);
data.comparison = load(files.comparison);
macroSample = load(files.macro);
data.macro = macroSample.result;
refSample = load(files.refs, 'refs');
data.refs = refSample.refs;
end

function record = saveFigure(fig, outputDir, name)
pngFile = fullfile(outputDir, char(name + ".png"));
pdfFile = fullfile(outputDir, char(name + ".pdf"));
exportgraphics(fig, pngFile, 'Resolution', 200);
exportgraphics(fig, pdfFile, 'ContentType', 'vector');
close(fig);
record = [name, string(pngFile), string(pdfFile)];
end

function record = exportOrDrawSystem(projectRoot, outputDir, systemName, name, fallback)
pngFile = fullfile(outputDir, char(name + ".png"));
pdfFile = fullfile(outputDir, char(name + ".pdf"));
success = false;
try
    if systemName == "stewart_platform_model"
        load_system(fullfile(projectRoot, 'matlab', 'stewart_platform_model.slx'));
    else
        load_system(fullfile(projectRoot, 'matlab', 'simscape_subsystems', 'stewart_strut.slx'));
    end
    print(['-s', char(systemName)], '-dpng', '-r200', pngFile);
    close_system(char(systemName), 0);
    success = isfile(pngFile);
catch
    if bdIsLoaded(char(systemName))
        close_system(char(systemName), 0);
    end
end
if success
    fig = fallback();
    exportgraphics(fig, pdfFile, 'ContentType', 'vector');
    close(fig);
else
    fig = fallback();
    exportgraphics(fig, pngFile, 'Resolution', 200);
    exportgraphics(fig, pdfFile, 'ContentType', 'vector');
    close(fig);
end
record = [name, string(pngFile), string(pdfFile)];
end

function fig = baseFigure(titleText, position)
if nargin < 2
    position = [80, 80, 1450, 900];
end
fig = figure('Color', 'w', 'Visible', 'off', 'Position', position);
sgtitle(fig, titleText, 'FontWeight', 'bold', 'FontSize', 15);
end

function fig = flowFigure(titleText, labels, colors, edges, styles)
fig = baseFigure(titleText, [80, 80, 1500, 760]);
ax = axes(fig, 'Position', [0.02, 0.04, 0.96, 0.88]);
axis(ax, [0, 1, 0, 1]); axis(ax, 'off'); hold(ax, 'on');
count = numel(labels);
columns = ceil(sqrt(count * 1.8));
rows = ceil(count / columns);
positions = zeros(count, 4);
for index = 1:count
    column = mod(index - 1, columns);
    row = floor((index - 1) / columns);
    positions(index, :) = [0.04 + column * (0.92 / columns), ...
        0.78 - row * (0.72 / max(rows - 1, 1)), 0.78 / columns, 0.12];
end
for index = 1:size(edges, 1)
    source = positions(edges(index, 1), :);
    target = positions(edges(index, 2), :);
    sourceCenter = [source(1) + source(3) / 2, source(2) + source(4) / 2];
    targetCenter = [target(1) + target(3) / 2, target(2) + target(4) / 2];
    vector = targetCenter - sourceCenter;
    startPoint = sourceCenter + 0.08 * vector;
    endPoint = targetCenter - 0.08 * vector;
    lineStyle = '-';
    if nargin >= 5 && numel(styles) >= index
        lineStyle = styles{index};
    end
    plot(ax, [startPoint(1), endPoint(1)], [startPoint(2), endPoint(2)], ...
        'LineStyle', lineStyle, 'LineWidth', 1.2, 'Color', [0.2, 0.2, 0.2]);
    plot(ax, endPoint(1), endPoint(2), 'o', 'MarkerSize', 3.5, ...
        'MarkerFaceColor', [0.2, 0.2, 0.2], 'MarkerEdgeColor', [0.2, 0.2, 0.2]);
end
for index = 1:count
    rectangle(ax, 'Position', positions(index, :), 'Curvature', 0.08, ...
        'FaceColor', colors(index, :), 'EdgeColor', [0.15, 0.15, 0.15], 'LineWidth', 1.2);
    text(ax, positions(index, 1) + positions(index, 3) / 2, ...
        positions(index, 2) + positions(index, 4) / 2, labels{index}, ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'Interpreter', 'none');
end
end

function fig = makeArchitectureFigure()
labels = {'参考轨迹与前馈力', 'Run02 位姿力控制器', 'Simscape Multibody', ...
    'Run03/Run04 理想腿长对照', '目标：全链路 Simscape', ...
    'PWM 位姿外环与力内环', 'PWM 高保真执行器', 'MATLAB 合成刚体动力学', ...
    '编码器与位姿测量', 'UKF 与灰箱+NARX'};
colors = palette([1, 2, 4, 2, 6, 2, 3, 4, 1, 5]);
edges = [1,2;2,3;1,4;4,3;3,5;6,7;7,8;8,9;9,10];
styles = {'-','-','-','-','--','-','-','-','-'};
fig = flowFigure('项目总体仿真架构：已验证链路与待统一链路', labels, colors, edges, styles);
end

function fig = makeGeometryFigure(model)
fig = baseFigure('Stewart 平台几何、坐标系与六条支链');
ax = axes(fig, 'Position', [0.08, 0.08, 0.84, 0.82]); hold(ax, 'on');
kin = sgpIK(model.qHome, model);
plot3(ax, [model.A(1, :), model.A(1, 1)], [model.A(2, :), model.A(2, 1)], ...
    [model.A(3, :), model.A(3, 1)], 'k-', 'LineWidth', 2);
Bworld = model.qHome(1:3) + kin.rB;
plot3(ax, [Bworld(1, :), Bworld(1, 1)], [Bworld(2, :), Bworld(2, 1)], ...
    [Bworld(3, :), Bworld(3, 1)], 'Color', [0.1, 0.45, 0.75], 'LineWidth', 2);
for index = 1:6
    plot3(ax, [model.A(1,index), Bworld(1,index)], [model.A(2,index), Bworld(2,index)], ...
        [model.A(3,index), Bworld(3,index)], 'Color', [0.9, 0.35, 0.1], 'LineWidth', 1.6);
    text(ax, model.A(1,index), model.A(2,index), model.A(3,index), sprintf(' A%d', index));
    text(ax, Bworld(1,index), Bworld(2,index), Bworld(3,index), sprintf(' B%d', model.legMap(index)));
end
quiver3(ax, 0,0,0, 0.25,0,0, 'r', 'LineWidth', 1.5);
quiver3(ax, 0,0,0, 0,0.25,0, 'g', 'LineWidth', 1.5);
quiver3(ax, 0,0,0, 0,0,0.25, 'b', 'LineWidth', 1.5);
grid(ax, 'on'); axis(ax, 'equal'); view(ax, 35, 23);
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
end

function fig = makeAnchorMappingFigure(model)
fig = baseFigure('固定平台与动平台铰点编号及支链连接映射');
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
ax = nexttile(layout);
plotClosed(ax, model.A, 'A', [0.15,0.15,0.15]); title(ax, '固定平台铰点 A_i');
ax = nexttile(layout);
plotClosed(ax, model.B, 'B', [0.1,0.45,0.75]); title(ax, '动平台铰点 B_i 与实际支链');
for index = 1:6
    text(ax, model.B(1,model.legMap(index)), model.B(2,model.legMap(index)), ...
        sprintf('\\leftarrow 腿%d', index), 'FontSize', 9);
end
end

function plotClosed(ax, points, prefix, color)
plot(ax, [points(1,:),points(1,1)], [points(2,:),points(2,1)], '-o', ...
    'Color', color, 'LineWidth', 1.5, 'MarkerFaceColor', color);
hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
for index = 1:6
    text(ax, points(1,index), points(2,index), sprintf(' %s%d', prefix, index));
end
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
end

function fig = makeSingleLegFigure(model)
fig = baseFigure('单条支链矢量关系');
ax = axes(fig, 'Position', [0.08,0.08,0.84,0.82]); hold(ax, 'on');
kin = sgpIK(model.qHome, model);
i = 1; A = model.A(:,i); p = model.qHome(1:3); r = kin.rB(:,i); B = p + r;
plot3(ax, A(1),A(2),A(3),'ko','MarkerFaceColor','k');
plot3(ax, p(1),p(2),p(3),'bo','MarkerFaceColor','b');
plot3(ax, B(1),B(2),B(3),'ro','MarkerFaceColor','r');
quiver3(ax,p(1),p(2),p(3),r(1),r(2),r(3),0,'b','LineWidth',2);
quiver3(ax,A(1),A(2),A(3),kin.s(1,i),kin.s(2,i),kin.s(3,i),0,'r','LineWidth',2);
text(ax,A(1),A(2),A(3),' A_i'); text(ax,p(1),p(2),p(3),' p');
text(ax,B(1),B(2),B(3),' p+RB_i'); text(ax,mean([A(1),B(1)]),mean([A(2),B(2)]),mean([A(3),B(3)]),' s_i=L_i u_i');
grid(ax,'on'); axis(ax,'equal'); view(ax,35,25);
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
end

function fig = makeKinematicsFlowFigure()
labels = {'平台位姿 q', '旋转矩阵 R', '支链向量 s_i', '长度 L_i 与方向 u_i', ...
    'Jacobian J_v / J_q', '腿速度 Ldot', '六腿轴向力 F', ...
    '广义驱动力 J_v^T F', '质量矩阵 H 与偏置力', '平台加速度 qdd'};
colors = palette([1,1,1,1,4,4,3,3,4,4]);
edges = [1,2;2,3;3,4;4,5;1,5;5,6;7,8;5,8;8,10;9,10;1,9];
fig = flowFigure('运动学、Jacobian 与动力学计算链', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeTrajectoryKinematicsFigure(refs, model)
t = refs.t(:);
count = numel(t); L = zeros(count,6); Ld = zeros(count,6); sigma = zeros(count,1); condJ = zeros(count,1);
for k = 1:count
    kin = sgpIK(refs.q(:,k), model); jac = sgpJacobian(refs.q(:,k), model);
    L(k,:) = kin.L.'; Ld(k,:) = (jac.Jq * refs.qd(:,k)).';
    sigma(k) = jac.sigmaMin; condJ(k) = jac.condJ;
end
fig = baseFigure('典型轨迹中的腿长、腿速度与奇异性指标', [80,80,1450,1000]);
layout = tiledlayout(fig,3,1,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,t,L,'LineWidth',0.9); grid(ax,'on'); ylabel(ax,'腿长 (m)');
ax=nexttile(layout); plot(ax,t,Ld,'LineWidth',0.9); grid(ax,'on'); ylabel(ax,'腿速度 (m/s)');
ax=nexttile(layout); yyaxis(ax,'left'); plot(ax,t,sigma,'LineWidth',1.2); ylabel(ax,'最小奇异值');
yyaxis(ax,'right'); plot(ax,t,condJ,'LineWidth',1.2); ylabel(ax,'条件数'); grid(ax,'on'); xlabel(ax,'时间 (s)');
end

function fig = makeSimscapeTopFigure()
labels = {'Reference', 'Controller', 'Force Feedforward', '反馈+前馈求和', ...
    'Stewart Platform', 'Payload', 'Ground', 'Relative Motion Sensor', ...
    'Simscape Configuration', 'simout 日志'};
colors = palette([1,2,1,2,4,4,4,5,6,1]);
edges = [1,2;2,4;3,4;4,5;6,5;7,5;5,8;8,2;5,10;8,10;9,5];
fig = flowFigure('stewart_platform_model.slx 顶层模块职责', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeSimscapePlatformFigure()
labels = {'固定平台', '支链1', '支链2', '支链3', '支链4', '支链5', '支链6', ...
    '动平台', '负载刚体', '相对运动/腿长/力测量'};
colors = palette([4,3,3,3,3,3,3,4,4,5]);
edges = [1,2;1,3;1,4;1,5;1,6;1,7;2,8;3,8;4,8;5,8;6,8;7,8;8,9;2,10;8,10];
fig = flowFigure('Stewart Platform 子系统层级示意', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeStrutFigure()
labels = {'固定平台连接', '理想万向铰', '固定端杆件', 'Prismatic Joint', ...
    '移动端杆件', '理想球铰', '动平台连接', '轴向驱动力/PWM Variant', '腿长与腿速度测量'};
colors = palette([4,4,4,3,4,4,4,3,5]);
edges = [1,2;2,3;3,4;4,5;5,6;6,7;8,4;4,9];
fig = flowFigure('单条支链 stewart_strut.slx 结构', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeParameterMappingFigure()
labels = {'buildOptModelCustom', '几何 A/B/legMap', '质量、质心与惯量', ...
    '约束与重力', 'buildSimscapeLengthControlData', 'stewart', 'payload', ...
    'ground/disturbances', '初始腿长一致性', '平行轴定理一致性', 'Simscape 基础工作区'};
colors = palette([1,1,1,1,2,4,4,4,5,5,4]);
edges = [1,2;1,3;1,4;2,5;3,5;4,5;5,6;5,7;5,8;6,9;6,10;7,10;6,11;7,11;8,11];
fig = flowFigure('MATLAB 参数结构到 Simscape 模块参数的映射', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeReferenceReconstructionFigure(refs)
tNode = refs.t(:); qNode = refs.q(1,:).';
t = linspace(tNode(1), tNode(end), 1200).';
linear = interp1(tNode,qNode,t,'linear');
hermite = pchip(tNode,qNode,t);
fig = baseFigure('参考节点到规则时间网格的重建示意');
layout=tiledlayout(fig,2,1,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,t,1e3*linear,'--',t,1e3*hermite,'LineWidth',1.2); hold(ax,'on');
plot(ax,tNode,1e3*qNode,'ko','MarkerFaceColor','k'); grid(ax,'on'); ylabel(ax,'X (mm)');
legend(ax,'线性连接示意','平滑重建示意','原始节点','Location','best');
ax=nexttile(layout); plot(ax,t(2:end),diff(linear)./diff(t),'--',t(2:end),diff(hermite)./diff(t),'LineWidth',1.2);
grid(ax,'on'); xlabel(ax,'时间 (s)'); ylabel(ax,'速度 (m/s)'); title(ax,'平滑重建避免节点速度跳变');
end

function fig = makeReferenceDataflowFigure()
labels = {'轨迹 MAT 文件', '节点 t/q/qd/Fleg/L', 'Hermite 位姿重建', ...
    'IK 与 Jacobian 同步计算', 'references.r', 'references.rL/rLd', ...
    'references.uFF', 'From Workspace', 'Simscape Controller/Plant'};
colors = palette([1,1,2,2,1,1,1,4,4]);
edges = [1,2;2,3;3,4;3,5;4,6;2,7;5,8;6,8;7,8;8,9];
fig = flowFigure('参考信号从 MAT 文件进入 Simscape 的数据流', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeRun02ControlFigure()
labels = {'q_ref, qd_ref', '位姿误差', '笛卡尔 PIDF K_x', 'J_v^{-T} 腿力映射', ...
    '反馈腿力 F_fb', '前馈腿力 F_FF', '总腿力 u', 'Simscape 多体对象', '相对位姿 X_r'};
colors = palette([1,2,2,2,2,1,3,4,5]);
edges = [1,2;9,2;2,3;3,4;4,5;5,7;6,7;7,8;8,9];
fig = flowFigure('Run02：理想六腿力输入位姿控制结构', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeRun02DesignFigure()
labels = {'关闭重力、清零前馈', 'Simscape 力到位姿线性化', 'J_v^{-T} 转为笛卡尔对象', ...
    '六个对角通道 pidtune', '平移/旋转增益缩放', '闭环极点稳定性检查', ...
    '恢复重力与前馈', '完整轨迹仿真'};
colors = palette([6,4,2,2,2,5,4,4]);
edges = [(1:7).',(2:8).'];
fig = flowFigure('Run02 控制器线性化与整定流程', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeRun02ResultFigure(report)
fig=baseFigure('Run02 Simscape 理想力源完整轨迹结果',[80,80,1450,1050]);
layout=tiledlayout(fig,2,2,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,report.poseTime,1e3*report.poseError(:,1:3),'LineWidth',0.9); grid(ax,'on'); ylabel(ax,'平移误差 (mm)');
ax=nexttile(layout); plot(ax,report.poseTime,rad2deg(report.poseError(:,4:6)),'LineWidth',0.9); grid(ax,'on'); ylabel(ax,'旋转误差 (deg)');
ax=nexttile(layout); plot(ax,report.time,1e3*report.lengthError,'LineWidth',0.9); grid(ax,'on'); ylabel(ax,'腿长误差 (mm)'); xlabel(ax,'时间 (s)');
ax=nexttile(layout); plot(ax,report.forceTime,report.controlForce,'LineWidth',0.9); grid(ax,'on'); ylabel(ax,'驱动力 (N)'); xlabel(ax,'时间 (s)');
end

function fig = makeControlComparisonFigure()
labels = {'Run02：位姿误差', '笛卡尔动态反馈', '理想腿力源', ...
    'Run03：腿长误差', '位置P+速度PIDF', '理想腿长伺服', ...
    'Run04：位姿误差', 'J_q 腿长修正', 'Run03 串级伺服'};
colors = palette([1,2,3,1,2,3,1,2,3]);
edges = [1,2;2,3;4,5;5,6;7,8;8,9];
fig = flowFigure('Run02、Run03、Run04 控制结构对照', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeThreeRunResultFigure(data)
names={'Run02 理想力输入','Run03 理想腿长串级','Run04 位姿反馈腿长'};
reports={data.run02.report,data.run03.report,data.run04.report};
translation=zeros(3,2); rotation=zeros(3,2); length=zeros(3,2);
for i=1:3
    r=reports{i};
    translation(i,:)=[1e3*sqrt(mean(r.poseError(:,1:3).^2,'all')),1e3*max(abs(r.poseError(:,1:3)),[],'all')];
    rotation(i,:)=[rad2deg(sqrt(mean(r.poseError(:,4:6).^2,'all'))),rad2deg(max(abs(r.poseError(:,4:6)),[],'all'))];
    length(i,:)=[1e3*sqrt(mean(r.lengthError.^2,'all')),1e3*max(abs(r.lengthError),[],'all')];
end
fig=baseFigure('Run02、Run03、Run04 系统级结果对比',[80,80,1450,850]);
layout=tiledlayout(fig,1,3,'TileSpacing','compact');
ax=nexttile(layout); bar(ax,translation); grid(ax,'on'); title(ax,'平移误差'); ylabel(ax,'mm'); xticklabels(ax,names); xtickangle(ax,20); legend(ax,'RMS','峰值');
ax=nexttile(layout); bar(ax,rotation); grid(ax,'on'); title(ax,'旋转误差'); ylabel(ax,'deg'); xticklabels(ax,names); xtickangle(ax,20);
ax=nexttile(layout); bar(ax,length); grid(ax,'on'); title(ax,'腿长误差'); ylabel(ax,'mm'); xticklabels(ax,names); xtickangle(ax,20);
end

function fig = makePwmChainFigure()
labels = {'有符号 PWM', '死区', '平均 H 桥电压', 'R-L 电气动态与反电动势', ...
    '电机转矩', '减速器与丝杠', '电磁轴向力', 'Stribeck/黏性摩擦', '力饱和', '真实腿力'};
colors = palette([1,3,3,3,3,3,3,6,6,4]);
edges = [(1:9).',(2:10).'];
fig = flowFigure('PWM 高保真执行器完整物理信号链', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makePwmCharacteristicFigure(teacher)
commands=linspace(-teacher.pwmMax,teacher.pwmMax,161); speeds=[0,0.15,0.30]; force=zeros(numel(commands),3);
for j=1:3
    for k=1:numel(commands)
        state=initializeHighFidelityPwmActuatorState(teacher); cmd=zeros(6,1); cmd(1)=commands(k); v=zeros(6,1); v(1)=speeds(j);
        for s=1:100, [state,out]=stepHighFidelityPwmActuator(state,cmd,v,teacher); end
        force(k,j)=out.force(1);
    end
end
fig=baseFigure('PWM–力静态特性、速度影响与六轴死区',[80,80,1450,800]);
layout=tiledlayout(fig,1,2,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,commands,force,'LineWidth',1.4); hold(ax,'on'); xline(ax,teacher.deadzonePwm(1),'k--'); xline(ax,-teacher.deadzonePwm(1),'k--'); grid(ax,'on');
xlabel(ax,'PWM'); ylabel(ax,'稳态输出力 (N)'); legend(ax,compose('腿速 %.2f m/s',speeds));
ax=nexttile(layout); bar(ax,teacher.deadzonePwm); grid(ax,'on'); xlabel(ax,'执行器轴号'); ylabel(ax,'死区 PWM');
end

function fig = makeFrictionFigure(teacher)
v=linspace(-0.35,0.35,501).'; friction=zeros(size(v)); output=zeros(size(v));
for k=1:numel(v)
    state=initializeHighFidelityPwmActuatorState(teacher); cmd=zeros(6,1); cmd(1)=2600; speed=zeros(6,1); speed(1)=v(k);
    for s=1:100, [state,out]=stepHighFidelityPwmActuator(state,cmd,speed,teacher); end
    friction(k)=out.frictionForce(1); output(k)=out.force(1);
end
fig=baseFigure('低速摩擦与输出力分解');
layout=tiledlayout(fig,1,2,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,v,friction,'LineWidth',1.4); grid(ax,'on'); xlabel(ax,'腿速度 (m/s)'); ylabel(ax,'摩擦力 (N)'); title(ax,'Stribeck 与黏性摩擦');
ax=nexttile(layout); plot(ax,v,output,'LineWidth',1.4); grid(ax,'on'); xlabel(ax,'腿速度 (m/s)'); ylabel(ax,'最终输出力 (N)'); title(ax,'固定 PWM 下的速度影响');
end

function fig = makeSwitchingFigure(teacher)
sample=validateAveragePwmActuatorAgainstSwitching(teacher,struct('axisIndex',1,'duty',0.7,'duration',0.15,'legSpeed',0.05));
fig=baseFigure('平均 PWM 模型与 5 kHz 开关级模型校验');
layout=tiledlayout(fig,1,2,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,linspace(0,.15,numel(sample.switchingForce)),sample.switchingForce,'Color',[.75,.75,.75]); hold(ax,'on');
plot(ax,linspace(0,.15,numel(sample.averageForce)),sample.averageForce,'b-o','LineWidth',1.2); grid(ax,'on'); xlabel(ax,'时间 (s)'); ylabel(ax,'力 (N)'); legend(ax,'开关级瞬时力','平均值模型');
ax=nexttile(layout); bar(ax,[sample.metrics.meanSwitchingForce,sample.metrics.meanAverageForce]); grid(ax,'on'); ylabel(ax,'尾段平均力 (N)'); xticklabels(ax,{'开关级','平均值'}); title(ax,sprintf('相对误差 %.4f%%',100*sample.metrics.relativeMeanForceError));
end

function fig = makeIdentificationFlowFigure()
labels = {'多正弦+随机阶跃 PWM', '平台可实现位姿运动', '编码器与位姿采集', ...
    '高保真物理教师', '真实力标签（仅训练/评价）', '60/20/20 数据划分', ...
    '灰箱参数优化', '残差 NARX 结构选择', '自由运行与闭环精炼', '在线力估计器'};
colors = palette([1,1,5,4,6,1,2,2,2,5]);
edges = [1,4;2,3;2,4;4,5;1,6;3,6;5,6;6,7;7,8;8,9;9,10];
fig = flowFigure('辨识数据生成、训练与验证流程', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeCoverageFigure(dataset)
fig=baseFigure('辨识数据在 PWM–腿速度工况平面上的覆盖');
layout=tiledlayout(fig,1,2,'TileSpacing','compact');
ax=nexttile(layout); histogram2(ax,dataset.pwm(:),dataset.legSpeed(:), ...
    'NumBins',[45,40],'DisplayStyle','tile','ShowEmptyBins','off');
colorbar(ax); grid(ax,'on'); xlabel(ax,'PWM'); ylabel(ax,'腿速度 (m/s)'); title(ax,'六轴总体覆盖密度');
ax=nexttile(layout); scatter(ax,dataset.pwm(:,1),dataset.legSpeed(:,1),8,dataset.trueForce(:,1),'filled'); colorbar(ax); grid(ax,'on'); xlabel(ax,'PWM'); ylabel(ax,'腿速度 (m/s)'); title(ax,'轴1颜色表示真实力');
end

function fig = makeGrayParameterFigure(identifier)
fit=identifier.identified.training.grayParameterFit;
fig=baseFigure('灰箱可辨识参数优化结果');
ax=axes(fig,'Position',[.08,.16,.86,.72]); bar(ax,fit.scale); yline(ax,1,'k--'); grid(ax,'on');
xticks(ax,1:numel(fit.parameterNames)); xticklabels(ax,fit.parameterNames); xtickangle(ax,25); ylabel(ax,'相对初值缩放系数');
title(ax,sprintf('低速加权目标函数：%.6g → %.6g',fit.objectiveBefore,fit.objectiveAfter));
end

function fig = makeIdentificationResultFigure(identifier)
dataset=identifier.dataset; report=identifier.report; idx=find(dataset.split.test); t=dataset.time(idx);
fig=baseFigure('灰箱与灰箱+NARX 的独立测试集自由运行验证',[80,80,1450,950]);
layout=tiledlayout(fig,2,2,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,t,dataset.trueForce(idx,1),'k',t,report.grayForce(idx,1),'--',t,report.estimatedForce(idx,1),'LineWidth',1); grid(ax,'on'); ylabel(ax,'力 (N)'); legend(ax,'真实力','灰箱','灰箱+NARX');
ax=nexttile(layout); plot(ax,t,report.grayForce(idx,1)-dataset.trueForce(idx,1),'--',t,report.estimatedForce(idx,1)-dataset.trueForce(idx,1),'LineWidth',1); grid(ax,'on'); ylabel(ax,'误差 (N)'); legend(ax,'灰箱','灰箱+NARX');
ax=nexttile(layout); scatter(ax,dataset.trueForce(idx,:),report.estimatedForce(idx,:),6,'filled'); hold(ax,'on'); plot(ax,[-2400,2400],[-2400,2400],'r--'); axis(ax,'equal'); grid(ax,'on'); xlabel(ax,'真实力'); ylabel(ax,'估计力');
ax=nexttile(layout); histogram(ax,report.estimatedForce(idx,:)-dataset.trueForce(idx,:),50); grid(ax,'on'); xlabel(ax,'估计误差 (N)'); ylabel(ax,'样本数');
end

function fig = makeIdentificationMetricFigure(identifier)
dataset=identifier.dataset; report=identifier.report; mask=dataset.split.test; gray=zeros(6,1); narx=zeros(6,1); bias=zeros(6,1);
for i=1:6
    truth=dataset.trueForce(mask,i); gray(i)=sqrt(mean((report.grayForce(mask,i)-truth).^2))/4800; narx(i)=sqrt(mean((report.estimatedForce(mask,i)-truth).^2))/4800; bias(i)=mean(report.estimatedForce(mask,i)-truth);
end
fig=baseFigure('六轴辨识精度与偏差');
layout=tiledlayout(fig,1,2,'TileSpacing','compact');
ax=nexttile(layout); bar(ax,100*[gray,narx]); grid(ax,'on'); xlabel(ax,'轴号'); ylabel(ax,'NRMSE (%)'); legend(ax,'灰箱','灰箱+NARX');
ax=nexttile(layout); bar(ax,bias); grid(ax,'on'); xlabel(ax,'轴号'); ylabel(ax,'测试集平均偏差 (N)');
end

function fig = makeUkfFigure()
labels = {'状态 [q, qdot, bias]', '恒加速度运动预测', 'Sigma 点传播', ...
    '预测腿长 IK(q)', '编码器腿长测量', '位姿测量', '测量创新', ...
    'UKF 校正', '融合位姿/速度', '腿速度与力估计'};
colors = palette([5,5,5,4,1,1,6,5,5,5]);
edges = [1,2;2,3;3,4;4,7;5,7;6,7;7,8;8,9;9,10;8,1];
fig = flowFigure('偏置状态 UKF 的预测与校正流程', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makePoseBenchmarkFigure(data)
b=benchmarkPwmPoseEstimators(data.comparison.comparison.identified); t=b.t;
ke=b.kinematic.pose-b.truth.pose; ue=b.ukf.pose-b.truth.pose;
fig=baseFigure('运动学重建与偏置状态 UKF 对比',[80,80,1450,900]);
layout=tiledlayout(fig,2,2,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,t,1e3*vecnorm(ke(1:3,:),2,1),'--',t,1e3*vecnorm(ue(1:3,:),2,1),'LineWidth',1); grid(ax,'on'); ylabel(ax,'平移误差范数 (mm)'); legend(ax,'运动学重建','UKF');
ax=nexttile(layout); plot(ax,t,rad2deg(vecnorm(ke(4:6,:),2,1)),'--',t,rad2deg(vecnorm(ue(4:6,:),2,1)),'LineWidth',1); grid(ax,'on'); ylabel(ax,'旋转误差范数 (deg)');
ax=nexttile(layout); bar(ax,1e3*[b.metrics.kinematicTranslationRms,b.metrics.ukfTranslationRms;b.metrics.kinematicTranslationPeak,b.metrics.ukfTranslationPeak]); grid(ax,'on'); xticklabels(ax,{'平移RMS','平移峰值'}); ylabel(ax,'mm'); legend(ax,'运动学','UKF');
ax=nexttile(layout); bar(ax,[b.metrics.kinematicVelocityRms,b.metrics.ukfVelocityRms;b.metrics.kinematicLegSpeedRms,b.metrics.ukfLegSpeedRms]); grid(ax,'on'); xticklabels(ax,{'等效位姿速度RMS','腿速度RMS'}); ylabel(ax,'m/s');
end

function fig = makeForceAlignmentFigure(comparisonFile)
c=comparisonFile.comparison.identified; same=c.estimatedForce-c.trueForce; aligned=c.estimatedForce(:,2:end)-c.trueForce(:,1:end-1);
fig=baseFigure('在线力估计的一拍因果对齐');
layout=tiledlayout(fig,1,2,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,c.t,same(1,:),'Color',[.75,.3,.3]); hold(ax,'on'); plot(ax,c.t(2:end),aligned(1,:),'Color',[.1,.45,.75]); grid(ax,'on'); xlabel(ax,'时间 (s)'); ylabel(ax,'轴1误差 (N)'); legend(ax,'同拍误差','一拍对齐误差');
ax=nexttile(layout); bar(ax,[sqrt(mean(same.^2,'all')),sqrt(mean(aligned.^2,'all'))]); grid(ax,'on'); xticklabels(ax,{'同拍','一拍对齐'}); ylabel(ax,'六轴 RMS (N)');
end

function fig = makePwmClosedLoopFigure()
labels = {'参考位姿/速度与前馈力', 'UKF 融合位姿/速度', '位姿外环', '目标腿力', ...
    '灰箱逆模型前馈+力 PI', 'PWM 命令', '高保真 PWM 执行器', '真实腿力', ...
    '平台动力学', '编码器与位姿测量', '灰箱+NARX 力估计'};
colors = palette([1,5,2,2,2,3,3,4,4,1,5]);
edges = [1,3;2,3;3,4;4,5;11,5;5,6;6,7;7,8;8,9;9,10;10,2;6,11;2,11];
fig = flowFigure('PWM 辨识反馈完整闭环', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeForcePwmTimelineFigure(comparisonFile)
c=comparisonFile.comparison.identified; t=c.t;
fig=baseFigure('目标力、估计力、真实力与 PWM 命令的时间对应关系',[80,80,1450,1000]);
layout=tiledlayout(fig,3,1,'TileSpacing','compact');
ax=nexttile(layout); plot(ax,t,c.targetForce(1,:),'--',t,c.trueForce(1,:),'LineWidth',1); grid(ax,'on'); ylabel(ax,'力 (N)'); legend(ax,'目标力','真实力');
ax=nexttile(layout); plot(ax,t,c.estimatedForce(1,:),'-.' ,t,c.trueForce(1,:),'LineWidth',1); grid(ax,'on'); ylabel(ax,'力 (N)'); legend(ax,'估计力','真实力');
ax=nexttile(layout); plot(ax,t,c.pwm.','LineWidth',.8); grid(ax,'on'); xlabel(ax,'时间 (s)'); ylabel(ax,'PWM');
end

function fig = makeUtilizationFigure(data)
m=data.comparison.comparison.metrics; teacher=data.identifier.teacher;
c=data.comparison.comparison.identified;
values=[max(abs(c.pwm),[],'all')/teacher.pwmMax, ...
    max(abs(c.trueForce),[],'all')/max(teacher.forceLimit), ...
    m.identifiedForceTrackingNrmse/0.075];
fig=baseFigure('PWM、真实力与力跟踪误差的利用率');
ax=axes(fig,'Position',[.12,.18,.78,.68]); bar(ax,100*values); yline(ax,100,'r--','阈值/满量程'); grid(ax,'on'); ylabel(ax,'利用率或阈值比例 (%)');
xticklabels(ax,{'PWM满量程利用率','真实力限幅利用率','力跟踪NRMSE/验收阈值'});
end

function fig = makeExecutionFlowFigure()
labels = {'读取模型与轨迹', '参数映射/参考重建', '控制器线性化整定', 'Simscape Run02/03/04', ...
    'PWM 辨识数据生成', '灰箱+NARX 训练/精炼', 'PWM 双线闭环', '评价与硬验收', ...
    'MAT/CSV/PNG/PDF 输出', '31 项活动测试'};
colors = palette([1,2,2,4,4,2,4,5,1,5]);
edges = [1,2;2,3;3,4;1,5;5,6;6,7;4,8;7,8;8,9;9,10];
fig = flowFigure('从参数准备到仿真、评价和结果导出的执行流程', labels, colors, edges, repmat({'-'},1,size(edges,1)));
end

function fig = makeTestMatrixFigure()
groups={'模型/目录契约','Simscape映射与冒烟','Run02完整跟踪','Run03/04对照','PWM物理与辨识','PWM闭环与估计'};
counts=[4,4,4,11,4,4];
fig=baseFigure('31 项活动测试覆盖矩阵');
ax=axes(fig,'Position',[.10,.18,.84,.68]); barh(ax,counts,'FaceColor',[.1,.45,.75]); grid(ax,'on'); xlabel(ax,'测试数量'); yticklabels(ax,groups);
for i=1:numel(counts), text(ax,counts(i)+.15,i,sprintf('%d',counts(i))); end
end

function fig = makeMacroComparisonFigure(macro)
r=macro.runs; e={r.errorIdeal,r.errorPwmOracle,r.errorPwmIdentified}; labels={'Simscape理想力源','PWM真实反馈','PWM辨识反馈'}; colors=[.1,.45,.75;.2,.65,.35;.9,.35,.1];
fig=baseFigure('理想力源与 PWM 双线的宏观运动对比',[80,80,1450,1000]);
layout=tiledlayout(fig,2,2,'TileSpacing','compact');
ax=nexttile(layout); plot3(ax,r.reference(:,1),r.reference(:,2),r.reference(:,3),'k--','LineWidth',2); hold(ax,'on');
plot3(ax,r.ideal(:,1),r.ideal(:,2),r.ideal(:,3),'Color',colors(1,:)); plot3(ax,r.pwmOracle(:,1),r.pwmOracle(:,2),r.pwmOracle(:,3),'Color',colors(2,:)); plot3(ax,r.pwmIdentified(:,1),r.pwmIdentified(:,2),r.pwmIdentified(:,3),'Color',colors(3,:)); grid(ax,'on'); axis(ax,'equal'); view(ax,35,25); title(ax,'平台空间轨迹'); legend(ax,[{'参考轨迹'},labels]);
ax=nexttile(layout); for i=1:3, plot(ax,r.time,1e3*vecnorm(e{i}(:,1:3),2,2),'Color',colors(i,:),'LineWidth',1.1); hold(ax,'on'); end; grid(ax,'on'); ylabel(ax,'平移误差范数 (mm)'); legend(ax,labels);
ax=nexttile(layout); for i=1:3, plot(ax,r.time,rad2deg(vecnorm(e{i}(:,4:6),2,2)),'Color',colors(i,:),'LineWidth',1.1); hold(ax,'on'); end; grid(ax,'on'); ylabel(ax,'旋转误差范数 (deg)');
ax=nexttile(layout); vals=[[macro.metrics.translationRmsMm].',[macro.metrics.translationPeakMm].',[macro.metrics.rotationRmsDeg].',[macro.metrics.rotationPeakDeg].']; bar(ax,vals.'); grid(ax,'on'); xticklabels(ax,{'平移RMS','平移峰值','旋转RMS','旋转峰值'}); legend(ax,labels);
end

function fig = makeErrorSourceFigure(macro)
ideal=macro.metrics(1); oracle=macro.metrics(2); identified=macro.metrics(3);
translation=[ideal.translationRmsMm,oracle.translationRmsMm-ideal.translationRmsMm,identified.translationRmsMm-oracle.translationRmsMm];
rotation=[ideal.rotationRmsDeg,oracle.rotationRmsDeg-ideal.rotationRmsDeg,identified.rotationRmsDeg-oracle.rotationRmsDeg];
fig=baseFigure('系统误差来源分解与后续优化优先级');
layout=tiledlayout(fig,1,2,'TileSpacing','compact');
ax=nexttile(layout); bar(ax,1,translation,'stacked'); grid(ax,'on'); ylabel(ax,'平移 RMS 增量 (mm)'); xticks(ax,1); xticklabels(ax,{'系统链路'}); legend(ax,'理想基准','PWM物理执行器增量','估计与延迟增量');
ax=nexttile(layout); bar(ax,1,rotation,'stacked'); grid(ax,'on'); ylabel(ax,'旋转 RMS 增量 (deg)'); xticks(ax,1); xticklabels(ax,{'系统链路'}); legend(ax,'理想基准','PWM物理执行器增量','估计与延迟增量');
end

function colors = palette(indices)
table = [0.90,0.90,0.90; 0.72,0.84,0.96; 0.98,0.72,0.48; ...
    0.78,0.82,0.88; 0.65,0.88,0.70; 0.98,0.68,0.68];
colors = table(indices,:);
end
