%% exp3_discrete_second_order_pid.m
% 实验三：二阶对象数字控制系统设计
% 指导书给出离散对象 G(z)=(0.003z+0.001)/(z^2-1.94z+0.9418)。
% 主采样周期 Ts=0.01 s，遵循指导书与格式化报告的程序约定，避免使用冲突的 Ts=1。

clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
packageDir = fileparts(scriptDir);
addpath(scriptDir);
figureDir = fullfile(packageDir, 'outputs', 'figures');
tableDir = fullfile(packageDir, 'outputs', 'tables');
ensure_output_dirs(figureDir, tableDir);

%% 1. 离散二阶对象与数字 PID 控制器
Ts = 0.01;        % Ts=0.01 来自指导书/实验报告格式化约定
Gz = tf([0.003 0.001], [1 -1.94 0.9418], Ts);

Kp = 155;
Ki = 1.5;
Kd = 4.8;
z = tf('z', Ts);

% 采用报告中的离散 PID 形式：积分为 Ki*Ts/(z-1)，微分为 Kd*(z-1)/(Ts*z)。
Cz = Kp + Ki*Ts/(z - 1) + Kd*(z - 1)/(Ts*z);
Lz = Cz * Gz;
Tz = feedback(Lz, 1);

%% 2. 阶跃响应与无控制比较
t = (0:Ts:5).';
uStep = ones(size(t));
yStep = lsim(Tz, uStep, t);

fig = figure('Name', 'exp3_step_response');
stairs(t, uStep, 'k--', 'LineWidth', 1.0); hold on;
stairs(t, yStep, 'b-', 'LineWidth', 1.2);
legend('阶跃输入', 'PID 输出', 'Location', 'best');
common_plot_style(gca, '实验三 离散系统阶跃响应 (Ts=0.01 s)', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp3_step_response.png');

yUncontrolled = lsim(Gz, uStep, t);
fig = figure('Name', 'exp3_uncontrolled_comparison');
stairs(t, yStep, 'b-', 'LineWidth', 1.2); hold on;
stairs(t, yUncontrolled, 'r--', 'LineWidth', 1.2);
legend('PID 控制', '无控制对象', 'Location', 'best');
common_plot_style(gca, '实验三 PID 控制与无控制阶跃响应比较', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp3_uncontrolled_comparison.png');

%% 3. 正弦与方波输入测试
uSine = sin(2*pi*1*t);
ySine = lsim(Tz, uSine, t);
fig = figure('Name', 'exp3_sine_response');
plot(t, uSine, 'k--', 'LineWidth', 1.0); hold on;
stairs(t, ySine, 'r-', 'LineWidth', 1.2);
legend('输入', '输出', 'Location', 'best');
common_plot_style(gca, '实验三 正弦输入响应 (f=1 Hz)', '时间 (s)', '幅值');
save_figure(fig, figureDir, 'exp3_sine_response.png');

uSquare = sign(sin(2*pi*0.5*t));
uSquare(uSquare == 0) = 1;
ySquare = lsim(Tz, uSquare, t);
fig = figure('Name', 'exp3_square_response');
stairs(t, uSquare, 'k--', 'LineWidth', 1.0); hold on;
stairs(t, ySquare, 'g-', 'LineWidth', 1.2);
legend('输入', '输出', 'Location', 'best');
common_plot_style(gca, '实验三 方波输入响应 (f=0.5 Hz)', '时间 (s)', '幅值');
save_figure(fig, figureDir, 'exp3_square_response.png');

%% 4. PID 参数扫描
tSweep = (0:Ts:3).';
uSweep = ones(size(tSweep));

fig = figure('Name', 'exp3_kp_sweep');
kpValues = [50 155 300];
for idx = 1:numel(kpValues)
    Ttest = closed_loop_for_pid(Gz, z, Ts, kpValues(idx), Ki, Kd);
    y = lsim(Ttest, uSweep, tSweep);
    stairs(tSweep, y, 'LineWidth', 1.1); hold on;
end
legend(compose('Kp=%g', kpValues), 'Location', 'best');
common_plot_style(gca, '实验三 Kp 对离散阶跃响应的影响 (Ki=1.5, Kd=4.8)', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp3_kp_sweep.png');

fig = figure('Name', 'exp3_ki_sweep');
kiValues = [0.5 1.5 5];
for idx = 1:numel(kiValues)
    Ttest = closed_loop_for_pid(Gz, z, Ts, Kp, kiValues(idx), Kd);
    y = lsim(Ttest, uSweep, tSweep);
    stairs(tSweep, y, 'LineWidth', 1.1); hold on;
end
legend(compose('Ki=%g', kiValues), 'Location', 'best');
common_plot_style(gca, '实验三 Ki 对离散阶跃响应的影响 (Kp=155, Kd=4.8)', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp3_ki_sweep.png');

fig = figure('Name', 'exp3_kd_sweep');
kdValues = [1 4.8 10];
for idx = 1:numel(kdValues)
    Ttest = closed_loop_for_pid(Gz, z, Ts, Kp, Ki, kdValues(idx));
    y = lsim(Ttest, uSweep, tSweep);
    stairs(tSweep, y, 'LineWidth', 1.1); hold on;
end
legend(compose('Kd=%g', kdValues), 'Location', 'best');
common_plot_style(gca, '实验三 Kd 对离散阶跃响应的影响 (Kp=155, Ki=1.5)', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp3_kd_sweep.png');

%% 5. Z 域根轨迹与闭环零极点（手动画单位圆，不依赖 zgrid）
fig = figure('Name', 'exp3_root_locus');
[rootData, ~] = rlocus(Gz);
unitHandle = plot_unit_circle(); hold on;
branchHandles = plot(real(rootData.'), imag(rootData.'), 'b-', 'LineWidth', 1.5);
plantPoles = pole(Gz);
plantZeros = zero(Gz);
poleHandle = plot(real(plantPoles), imag(plantPoles), 'kx', 'MarkerSize', 11, 'LineWidth', 1.8);
zeroHandle = plot(real(plantZeros), imag(plantZeros), 'bo', 'MarkerSize', 9, 'LineWidth', 1.6);
xline(0, 'k:', 'LineWidth', 0.9);
yline(0, 'k:', 'LineWidth', 0.9);
axis equal;
xlim([-1.2 1.2]);
ylim([-1.2 1.2]);
legend([unitHandle, branchHandles(1), poleHandle, zeroHandle], ...
    {'单位圆', 'G(z) 根轨迹', '开环极点', '开环零点'}, 'Location', 'best');
common_plot_style(gca, '实验三 离散对象 G(z) 的 Z 域根轨迹（含单位圆）', '实轴', '虚轴');
save_figure(fig, figureDir, 'exp3_root_locus.png');

fig = figure('Name', 'exp3_pole_zero_unit_circle');
plot_unit_circle(); hold on;
mark_poles_zeros(Tz);
axis equal;
common_plot_style(gca, '实验三 闭环零极点分布（含单位圆）', '实轴', '虚轴');
save_figure(fig, figureDir, 'exp3_pole_zero_unit_circle.png');

%% 6. 指标表输出
pidMetrics = metric_row('PID_closed_loop', Kp, Ki, Kd, yStep, t, 1);
openMetrics = metric_row('Uncontrolled_plant', NaN, NaN, NaN, yUncontrolled, t, 1);
metricsTable = struct2table([pidMetrics; openMetrics]);
metricsTable.SampleTime_s = Ts * ones(height(metricsTable), 1);
write_table_safely(metricsTable, tableDir, 'exp3_metrics.csv');

fprintf('实验三完成：图像与指标表已写出。\n');

%% 本脚本局部工具函数
function ensure_output_dirs(figureDir, tableDir)
if exist(figureDir, 'dir') ~= 7
    mkdir(figureDir);
end
if exist(tableDir, 'dir') ~= 7
    mkdir(tableDir);
end
end

function Tclosed = closed_loop_for_pid(Gz, z, Ts, Kp, Ki, Kd)
Cz = Kp + Ki*Ts/(z - 1) + Kd*(z - 1)/(Ts*z);
Tclosed = feedback(Cz * Gz, 1);
end

function h = plot_unit_circle()
theta = linspace(0, 2*pi, 500);
h = plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1.0);
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
    legend('单位圆', '极点', '虚轴', '实轴', 'Location', 'best');
else
    legend('单位圆', '极点', '零点', '虚轴', '实轴', 'Location', 'best');
end
end

function row = metric_row(labelText, Kp, Ki, Kd, y, t, targetValue)
m = compute_step_metrics(y, t, targetValue, labelText);
row = struct();
row.Method = char(labelText);
row.Kp = Kp;
row.Ki = Ki;
row.Kd = Kd;
row.RiseTime = m.RiseTime;
row.PeakTime = m.PeakTime;
row.Overshoot = m.Overshoot;
row.SettlingTime = m.SettlingTime;
row.SteadyStateValue = m.SteadyStateValue;
row.SteadyStateError = m.SteadyStateError;
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


function write_table_safely(tableData, tableDir, fileName)
targetFile = fullfile(tableDir, fileName);
try
    writetable(tableData, targetFile);
    fprintf('已写出表格: %s\n', targetFile);
catch ME
    warning('exp3:WriteTableFailed', '写表失败: %s', ME.message);
end
end
