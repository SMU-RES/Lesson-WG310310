%% exp1_digital_pid.m
% 实验一：数字 PID 参数的整定
% 依据《计算机控制系统》实验指导书，被控对象为 G(s)=133/(s^2+25s)。
% 参考整理后的实验报告程序组织方式，给出闭环、开环、多输入、频域和参数整定图。

clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
packageDir = fileparts(scriptDir);
addpath(scriptDir);
figureDir = fullfile(packageDir, 'outputs', 'figures');
tableDir = fullfile(packageDir, 'outputs', 'tables');
ensure_output_dirs(figureDir, tableDir);

%% 1. 建立连续对象与 PID 控制器
s = tf('s');
G = 133 / (s^2 + 25*s);

% 指导书要求整定 PID 三参数；此处采用报告中整理后的稳定参数。
Kp = 15;
Ki = 2;
Kd = 0.5;
C = Kp + Ki/s + Kd*s;
L = C * G;
Tcl = feedback(L, 1);

%% 2. 闭环阶跃响应
tStep = 0:0.001:8;
yClosedStep = squeeze(step(Tcl, tStep));
fig = figure('Name', 'exp1_step_response');
plot(tStep, yClosedStep, 'b-', 'LineWidth', 1.6); hold on;
yline(1, 'k--', 'LineWidth', 1.0);
legend('闭环输出', '给定值', 'Location', 'best');
common_plot_style(gca, '实验一 PID 闭环阶跃响应', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp1_step_response.png');

%% 3. 开环响应（无控制对象含积分环节，阶跃下呈斜坡发散趋势）
yOpenStep = squeeze(step(G, tStep));
fig = figure('Name', 'exp1_open_loop_response');
plot(tStep, yOpenStep, 'b-', 'LineWidth', 1.6);
common_plot_style(gca, '实验一 开环对象阶跃响应（无控制）', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp1_open_loop_response.png');

%% 4. 正弦、方波、锯齿波输入测试
t = (0:0.005:8).';
amplitude = 2;
frequency = 0.5;

uSine = amplitude * sin(2*pi*frequency*t);
ySine = lsim(Tcl, uSine, t);
fig = figure('Name', 'exp1_sine_response');
plot(t, uSine, 'k--', 'LineWidth', 1.0); hold on;
plot(t, ySine, 'b-', 'LineWidth', 1.5);
legend('输入', '输出', 'Location', 'best');
common_plot_style(gca, '实验一 正弦输入跟踪响应', '时间 (s)', '幅值');
save_figure(fig, figureDir, 'exp1_sine_response.png');

uSquare = amplitude * sign(sin(2*pi*frequency*t));
uSquare(uSquare == 0) = amplitude;
ySquare = lsim(Tcl, uSquare, t);
fig = figure('Name', 'exp1_square_response');
plot(t, uSquare, 'k--', 'LineWidth', 1.0); hold on;
plot(t, ySquare, 'r-', 'LineWidth', 1.5);
legend('输入', '输出', 'Location', 'best');
common_plot_style(gca, '实验一 方波输入跟踪响应', '时间 (s)', '幅值');
save_figure(fig, figureDir, 'exp1_square_response.png');

% 不调用 sawtooth 等额外函数，直接用 mod 构造锯齿波，增强可运行性。
uSaw = amplitude * (2*mod(frequency*t, 1) - 1);
ySaw = lsim(Tcl, uSaw, t);
fig = figure('Name', 'exp1_sawtooth_response');
plot(t, uSaw, 'k--', 'LineWidth', 1.0); hold on;
plot(t, ySaw, 'g-', 'LineWidth', 1.5);
legend('输入', '输出', 'Location', 'best');
common_plot_style(gca, '实验一 锯齿波输入跟踪响应', '时间 (s)', '幅值');
save_figure(fig, figureDir, 'exp1_sawtooth_response.png');

%% 5. 频域与稳定性分析：Bode、Nyquist、根轨迹、零极点
[gainMargin, phaseMargin, gainCrossFreq, phaseCrossFreq] = margin(L);

fig = figure('Name', 'exp1_bode');
margin(L);
common_plot_style(fig);
save_figure(fig, figureDir, 'exp1_bode.png');

fig = figure('Name', 'exp1_nyquist');
[nyqReal, nyqImag] = nyquist(L);
nyqReal = squeeze(nyqReal);
nyqImag = squeeze(nyqImag);
plot(nyqReal, nyqImag, 'b-', 'LineWidth', 1.5); hold on;
plot(nyqReal, -nyqImag, 'b--', 'LineWidth', 1.0);
plot(-1, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 1.6);
legend('正频率', '负频率对称支', '(-1,j0)', 'Location', 'best');
common_plot_style(gca, '实验一 开环 Nyquist 图', '实轴', '虚轴');
save_figure(fig, figureDir, 'exp1_nyquist.png');

fig = figure('Name', 'exp1_root_locus');
[rootData, ~] = rlocus(L);
plot(real(rootData.'), imag(rootData.'), 'b-', 'LineWidth', 1.2); hold on;
mark_poles_zeros(L);
common_plot_style(gca, '实验一 开环根轨迹 C(s)G(s)', '实轴', '虚轴');
save_figure(fig, figureDir, 'exp1_root_locus.png');

fig = figure('Name', 'exp1_pole_zero');
mark_poles_zeros(Tcl);
common_plot_style(gca, '实验一 闭环零极点分布', '实轴', '虚轴');
save_figure(fig, figureDir, 'exp1_pole_zero.png');

%% 6. PID 参数扫描：分别改变 Kp、Ki、Kd
tSweep = 0:0.001:6;

fig = figure('Name', 'exp1_kp_sweep');
kpValues = [5 15 30];
for idx = 1:numel(kpValues)
    Ctest = kpValues(idx) + Ki/s + Kd*s;
    y = step(feedback(Ctest*G, 1), tSweep);
    plot(tSweep, y, 'LineWidth', 1.4); hold on;
end
legend(compose('Kp=%g', kpValues), 'Location', 'best');
common_plot_style(gca, '实验一 Kp 对阶跃响应的影响 (Ki=2, Kd=0.5)', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp1_kp_sweep.png');

fig = figure('Name', 'exp1_ki_sweep');
kiValues = [0.5 2 5];
for idx = 1:numel(kiValues)
    Ctest = Kp + kiValues(idx)/s + Kd*s;
    y = step(feedback(Ctest*G, 1), tSweep);
    plot(tSweep, y, 'LineWidth', 1.4); hold on;
end
legend(compose('Ki=%g', kiValues), 'Location', 'best');
common_plot_style(gca, '实验一 Ki 对阶跃响应的影响 (Kp=15, Kd=0.5)', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp1_ki_sweep.png');

fig = figure('Name', 'exp1_kd_sweep');
kdValues = [0.1 0.5 2];
for idx = 1:numel(kdValues)
    Ctest = Kp + Ki/s + kdValues(idx)*s;
    y = step(feedback(Ctest*G, 1), tSweep);
    plot(tSweep, y, 'LineWidth', 1.4); hold on;
end
legend(compose('Kd=%g', kdValues), 'Location', 'best');
common_plot_style(gca, '实验一 Kd 对阶跃响应的影响 (Kp=15, Ki=2)', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp1_kd_sweep.png');

%% 7. 指标表输出
stepMetrics = compute_step_metrics(Tcl, tStep, 1, 'PID closed-loop');
gainMarginDb = to_db(gainMargin);
Metric = {'RiseTime'; 'PeakTime'; 'Overshoot'; 'SettlingTime'; ...
    'SteadyStateError'; 'PhaseMargin'; 'GainMargin'; 'GainCrossoverFrequency'; 'PhaseCrossoverFrequency'};
Symbol = {'tr'; 'tp'; 'sigma'; 'ts'; 'ess'; 'Pm'; 'Gm'; 'wc'; 'w180'};
Value = [stepMetrics.RiseTime; stepMetrics.PeakTime; stepMetrics.Overshoot; ...
    stepMetrics.SettlingTime; stepMetrics.SteadyStateError; phaseMargin; ...
    gainMarginDb; phaseCrossFreq; gainCrossFreq];
Unit = {'s'; 's'; '%'; 's'; ''; 'deg'; 'dB'; 'rad/s'; 'rad/s'};
exp1MetricsTable = table(Metric, Symbol, Value, Unit);
write_table_safely(exp1MetricsTable, tableDir, 'exp1_metrics.csv');

fprintf('实验一完成：图像与指标表已写出。\n');

%% 本脚本局部工具函数
function ensure_output_dirs(figureDir, tableDir)
if exist(figureDir, 'dir') ~= 7
    mkdir(figureDir);
end
if exist(tableDir, 'dir') ~= 7
    mkdir(tableDir);
end
end

function save_figure(fig, figureDir, fileName)
targetFile = fullfile(figureDir, fileName);
try
    set(fig, ...
        'Color', 'w', ...
        'InvertHardcopy', 'off', ...
        'Units', 'inches', ...
        'Position', [0.5 0.5 11.0 6.8], ...
        'PaperUnits', 'inches', ...
        'PaperPosition', [0 0 11.0 6.8], ...
        'PaperSize', [11.0 6.8]);
    drawnow;
    print(fig, targetFile, '-dpng', '-r300');
catch ME
    warning('report:FigureExportFailed', '300dpi 导出失败，改用 saveas: %s', ME.message);
    saveas(fig, targetFile);
end
fprintf('已写出图像: %s\n', targetFile);
end



function mark_poles_zeros(sys)
poles = pole(sys);
zeros = zero(sys);
plot(real(poles), imag(poles), 'kx', 'MarkerSize', 10, 'LineWidth', 1.6); hold on;
if ~isempty(zeros)
    plot(real(zeros), imag(zeros), 'bo', 'MarkerSize', 8, 'LineWidth', 1.4);
end
xline(0, 'k:', 'LineWidth', 0.9);
yline(0, 'k:', 'LineWidth', 0.9);
if isempty(zeros)
    legend('极点', '虚轴', '实轴', 'Location', 'best');
else
    legend('极点', '零点', '虚轴', '实轴', 'Location', 'best');
end
end

function write_table_safely(tableData, tableDir, fileName)
targetFile = fullfile(tableDir, fileName);
try
    writetable(tableData, targetFile);
    fprintf('已写出表格: %s\n', targetFile);
catch ME
    warning('exp1:WriteTableFailed', '写表失败: %s', ME.message);
end
end

function gainDb = to_db(gainValue)
if isinf(gainValue)
    gainDb = Inf;
elseif gainValue > 0
    gainDb = 20*log10(gainValue);
else
    gainDb = NaN;
end
end
