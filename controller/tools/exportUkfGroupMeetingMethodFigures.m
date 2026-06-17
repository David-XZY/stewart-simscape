function outputDir = exportUkfGroupMeetingMethodFigures(outputDir)
% exportUkfGroupMeetingMethodFigures - 导出组会汇报使用的 UKF 方法图
arguments
    outputDir string = ""
end

controllerRoot = fileparts(fileparts(mfilename('fullpath')));
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
if strlength(outputDir) == 0
    outputDir = fullfile(projectRoot, 'results', 'reports', 'ukf_group_meeting_20260615');
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

exportUkfPrinciple(fullfile(outputDir, '06_UKF如何工作.png'));
exportUkfConstruction(fullfile(outputDir, '07_本文如何构建UKF.png'));
fprintf('UKF 组会方法图已导出：%s\n', outputDir);
end

function exportUkfPrinciple(fileName)
colors = reportColors();
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [50, 50, 1800, 980]);
ax = axes(fig, 'Position', [0 0 1 1], 'Visible', 'off');
xlim(ax, [0 1]); ylim(ax, [0 1]);

text(ax, 0.04, 0.94, 'UKF 如何工作：非线性状态估计的预测—校正循环', ...
    'FontSize', 25, 'FontWeight', 'bold', 'Color', colors.dark);
text(ax, 0.04, 0.895, ...
    '核心思想：用一组代表性采样点传播均值与协方差，不需要对非线性模型求导', ...
    'FontSize', 15, 'Color', colors.muted);

box(ax, [0.04 0.59 0.17 0.20], colors.blueLight, colors.blue, ...
    '上一时刻状态', {'状态估计与不确定度'; '作为本次计算起点'});
box(ax, [0.27 0.59 0.18 0.20], colors.indigoLight, colors.indigo, ...
    '构造代表性采样点', {'围绕当前均值分布'; '同时表达状态不确定度'});
box(ax, [0.51 0.59 0.18 0.20], colors.orangeLight, colors.orange, ...
    '非线性预测', {'所有采样点通过运动模型'; '得到先验状态与先验不确定度'});
box(ax, [0.75 0.59 0.20 0.20], colors.greenLight, colors.green, ...
    '测量校正', {'比较预测测量与实际测量'; '根据可信度修正状态'});

arrow(ax, [0.21 0.27], [0.69 0.69], colors.blue);
arrow(ax, [0.45 0.51], [0.69 0.69], colors.indigo);
arrow(ax, [0.69 0.75], [0.69 0.69], colors.orange);

smallBox(ax, [0.53 0.37 0.14 0.10], colors.orangeLight, colors.orange, ...
    '运动模型输入', {'控制量或惯性信息'});
smallBox(ax, [0.77 0.37 0.16 0.10], colors.greenLight, colors.green, ...
    '实际传感器测量', {'用于约束累计漂移'});
arrow(ax, [0.60 0.60], [0.47 0.59], colors.orange);
arrow(ax, [0.85 0.85], [0.47 0.59], colors.green);

annotation(fig, 'arrow', [0.86 0.14], [0.57 0.57], ...
    'LineWidth', 2.2, 'Color', colors.muted, 'HeadWidth', 11, 'HeadLength', 11);
text(ax, 0.43, 0.525, '校正后的状态进入下一时刻，循环执行', ...
    'HorizontalAlignment', 'center', 'FontSize', 14, 'Color', colors.muted);

formulaBox(ax, [0.04 0.12 0.28 0.19], colors.blueLight, colors.blue, ...
    '预测阶段回答', {'根据运动规律，下一时刻可能在哪里？'; ...
    '模型越不确定，先验不确定度增长越快'});
formulaBox(ax, [0.36 0.12 0.28 0.19], colors.greenLight, colors.green, ...
    '校正阶段回答', {'当前传感器观测支持哪个状态？'; ...
    '测量越可信，校正作用越强'});
formulaBox(ax, [0.68 0.12 0.27 0.19], colors.indigoLight, colors.indigo, ...
    '最终输出', {'连续、平滑的状态估计'; ...
    '同时给出位置、姿态、速度与偏置'});

exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function exportUkfConstruction(fileName)
colors = reportColors();
fig = figure('Color', 'w', 'Visible', 'off', 'Position', [50, 50, 1900, 1120]);
ax = axes(fig, 'Position', [0 0 1 1], 'Visible', 'off');
xlim(ax, [0 1]); ylim(ax, [0 1]);

text(ax, 0.035, 0.955, '本文如何构建 UKF：相对腿长与 IMU 融合恢复平台状态', ...
    'FontSize', 25, 'FontWeight', 'bold', 'Color', colors.dark);
text(ax, 0.035, 0.918, ...
    '没有三轴位置直接测量；IMU 负责预测，六条相对腿长与三轴姿态负责校正', ...
    'FontSize', 15, 'Color', colors.muted);

% 左侧：输入
box(ax, [0.035 0.61 0.20 0.23], colors.orangeLight, colors.orange, ...
    '预测输入：IMU', {'三轴加速度'; '三轴角速度'; '扣除估计偏置后参与积分'});
box(ax, [0.035 0.30 0.20 0.23], colors.greenLight, colors.green, ...
    '校正测量：九维', {'六条相对腿长'; '三轴姿态角'; '不存在三轴位置直接测量'});

% 中间：状态
box(ax, [0.31 0.54 0.32 0.30], colors.indigoLight, colors.indigo, ...
    '十八维状态', {'平台位姿：三轴位置 + 三轴姿态'; ...
    '平台速度：三轴线速度 + 三轴姿态角速度'; ...
    '传感器偏置：三轴加速度计偏置 + 三轴陀螺仪偏置'});
formulaPanel(ax, [0.31 0.30 0.32 0.18], colors.indigoLight, colors.indigo, ...
    '状态初始化', {'初始位姿取回零锚点'; '初始速度与两类偏置估计置零'; ...
    '相对编码器只能恢复回零后的相对运动'});

% 右侧：预测与观测模型
formulaPanel(ax, [0.69 0.57 0.275 0.27], colors.orangeLight, colors.orange, ...
    '预测模型', {'加速度：姿态旋转到世界系并恢复重力'; ...
    '位置：上一位置 + 速度积分 + 加速度二次积分'; ...
    '姿态：角速度转换为姿态角速度后积分'; ...
    '偏置：按随机游走模型缓慢变化'});
formulaPanel(ax, [0.69 0.30 0.275 0.20], colors.greenLight, colors.green, ...
    '观测模型', {'相对腿长预测 = 逆运动学腿长 - 回零腿长'; ...
    '姿态预测 = 状态中的三轴姿态角'; ...
    '利用测量残差校正全部十八维状态'});

arrow(ax, [0.235 0.31], [0.725 0.725], colors.orange);
arrow(ax, [0.235 0.31], [0.415 0.415], colors.green);
arrow(ax, [0.63 0.69], [0.70 0.70], colors.orange);
arrow(ax, [0.63 0.69], [0.40 0.40], colors.green);

% 底部参数带
text(ax, 0.035, 0.255, '当前评估中的主要噪声与权重设置', ...
    'FontSize', 16, 'FontWeight', 'bold', 'Color', colors.dark);
parameterCell(ax, [0.035 0.055 0.21 0.15], colors.blueLight, colors.blue, ...
    '相对腿长', {'实际模拟噪声：0.5 毫米'; 'UKF 测量标准差：1 毫米'});
parameterCell(ax, [0.275 0.055 0.21 0.15], colors.greenLight, colors.green, ...
    '姿态角', {'分辨率：0.0055 度'; '测量与 UKF 标准差：0.1、0.1、0.5 度'});
parameterCell(ax, [0.515 0.055 0.21 0.15], colors.orangeLight, colors.orange, ...
    '加速度', {'噪声标准差：1 毫克重力加速度'; '偏置作为状态在线估计'});
parameterCell(ax, [0.755 0.055 0.21 0.15], colors.indigoLight, colors.indigo, ...
    '角速度', {'噪声标准差：0.07 度每秒'; '偏置作为状态在线估计'});

exportgraphics(fig, fileName, 'Resolution', 220);
close(fig);
end

function colors = reportColors()
colors.dark = [0.10 0.15 0.23];
colors.muted = [0.34 0.40 0.48];
colors.blue = [0.16 0.43 0.78];
colors.blueLight = [0.91 0.96 1.00];
colors.indigo = [0.36 0.32 0.72];
colors.indigoLight = [0.94 0.93 1.00];
colors.orange = [0.88 0.45 0.14];
colors.orangeLight = [1.00 0.95 0.89];
colors.green = [0.13 0.58 0.38];
colors.greenLight = [0.91 0.98 0.94];
end

function box(ax, position, faceColor, edgeColor, titleText, lines)
rectangle(ax, 'Position', position, 'Curvature', 0.06, ...
    'FaceColor', faceColor, 'EdgeColor', edgeColor, 'LineWidth', 2);
text(ax, position(1) + 0.018, position(2) + position(4) - 0.045, titleText, ...
    'FontSize', 17, 'FontWeight', 'bold', 'Color', edgeColor, ...
    'VerticalAlignment', 'top');
text(ax, position(1) + 0.018, position(2) + position(4) - 0.095, strjoin(lines, newline), ...
    'FontSize', 13.5, 'Color', [0.15 0.19 0.25], 'VerticalAlignment', 'top');
end

function smallBox(ax, position, faceColor, edgeColor, titleText, lines)
rectangle(ax, 'Position', position, 'Curvature', 0.06, ...
    'FaceColor', faceColor, 'EdgeColor', edgeColor, 'LineWidth', 1.7);
text(ax, position(1) + 0.012, position(2) + position(4) - 0.025, titleText, ...
    'FontSize', 13.5, 'FontWeight', 'bold', 'Color', edgeColor, 'VerticalAlignment', 'top');
text(ax, position(1) + 0.012, position(2) + 0.025, strjoin(lines, newline), ...
    'FontSize', 11.5, 'Color', [0.15 0.19 0.25], 'VerticalAlignment', 'bottom');
end

function formulaBox(ax, position, faceColor, edgeColor, titleText, lines)
rectangle(ax, 'Position', position, 'Curvature', 0.04, ...
    'FaceColor', faceColor, 'EdgeColor', edgeColor, 'LineWidth', 1.8);
text(ax, position(1) + 0.018, position(2) + position(4) - 0.035, titleText, ...
    'FontSize', 16, 'FontWeight', 'bold', 'Color', edgeColor, 'VerticalAlignment', 'top');
text(ax, position(1) + 0.018, position(2) + position(4) - 0.085, strjoin(lines, newline), ...
    'FontSize', 13, 'Color', [0.15 0.19 0.25], 'VerticalAlignment', 'top');
end

function formulaPanel(ax, position, faceColor, edgeColor, titleText, lines)
rectangle(ax, 'Position', position, 'Curvature', 0.04, ...
    'FaceColor', faceColor, 'EdgeColor', edgeColor, 'LineWidth', 1.8);
text(ax, position(1) + 0.015, position(2) + position(4) - 0.032, titleText, ...
    'FontSize', 15.5, 'FontWeight', 'bold', 'Color', edgeColor, 'VerticalAlignment', 'top');
text(ax, position(1) + 0.015, position(2) + position(4) - 0.075, strjoin(lines, newline), ...
    'FontSize', 12.2, 'Color', [0.15 0.19 0.25], 'VerticalAlignment', 'top');
end

function parameterCell(ax, position, faceColor, edgeColor, titleText, lines)
rectangle(ax, 'Position', position, 'Curvature', 0.04, ...
    'FaceColor', faceColor, 'EdgeColor', edgeColor, 'LineWidth', 1.6);
text(ax, position(1) + 0.012, position(2) + position(4) - 0.025, titleText, ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', edgeColor, 'VerticalAlignment', 'top');
text(ax, position(1) + 0.012, position(2) + position(4) - 0.065, strjoin(lines, newline), ...
    'FontSize', 11.5, 'Color', [0.15 0.19 0.25], 'VerticalAlignment', 'top');
end

function arrow(ax, x, y, color)
annotation(ancestor(ax, 'figure'), 'arrow', x, y, ...
    'LineWidth', 2.2, 'Color', color, 'HeadWidth', 10, 'HeadLength', 10);
end
