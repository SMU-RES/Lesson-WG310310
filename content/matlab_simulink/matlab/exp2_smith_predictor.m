%% exp2_smith_predictor.m
% 实验二：Smith 算法的运用
% 指导书采用一阶加纯迟后对象，PI 参数 Kp=40、Ki=0.022，阶跃输入取 100。
% 注意：本脚本主值 tau=1 s，遵循指导书/报告的实验二设置，避免使用冲突的 tau=100。

clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
packageDir = fileparts(scriptDir);
addpath(scriptDir);
figureDir = fullfile(packageDir, 'outputs', 'figures');
tableDir = fullfile(packageDir, 'outputs', 'tables');
ensure_output_dirs(figureDir, tableDir);

%% 1. 一阶加纯迟后对象与 PI 控制器
s = tf('s');
plantTimeConstant = 60;
tau = 1;                 % tau=1 来自实验指导书和格式化报告约定
stepAmplitude = 100;
Kp = 40;
Ki = 0.022;

G0 = 1 / (plantTimeConstant*s + 1);    % 无延迟模型
C = Kp + Ki/s;                         % PI 控制器
Lsmith = C * G0;
TsmithNoDelay = feedback(Lsmith, 1);   % Smith 等效闭环（特征方程不含纯滞后）

t = (0:0.1:800).';
uStep = stepAmplitude * ones(size(t));

%% 2. Smith 等效响应：预估输出与实际延迟输出
yPredicted = lsim(TsmithNoDelay, uStep, t);
yActual = apply_transport_delay(t, yPredicted, tau);

fig = figure('Name', 'exp2_smith_response');
plot(t, uStep, 'k--', 'LineWidth', 1.0); hold on;
plot(t, yPredicted, 'b-', 'LineWidth', 1.5);
plot(t, yActual, 'r-', 'LineWidth', 1.5);
legend('阶跃输入', '预估输出（无延迟）', '实际输出（含 tau=1 延迟）', 'Location', 'southeast');
common_plot_style(gca, '实验二 Smith 预估器等效响应', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp2_smith_response.png');

%% 3. 与传统 PI 控制比较：传统控制使用 Pade 近似延迟
padeOrder = 4;
[delayNum, delayDen] = pade(tau, padeOrder);
Dnominal = tf(delayNum, delayDen);
GdelayedPade = G0 * Dnominal;
Tconventional = minreal(feedback(C * GdelayedPade, 1), 1e-7);
yConventional = lsim(Tconventional, uStep, t);

fig = figure('Name', 'exp2_pi_comparison');
plot(t, yActual, 'r-', 'LineWidth', 1.5); hold on;
plot(t, yConventional, 'b--', 'LineWidth', 1.5);
plot(t, uStep, 'k:', 'LineWidth', 1.0);
legend('Smith 控制', '传统 PI（Pade 延迟）', '参考输入', 'Location', 'southeast');
common_plot_style(gca, '实验二 Smith 控制与传统 PI 控制比较', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp2_pi_comparison.png');

%% 4. 延迟鲁棒性：实际延迟为标称值的 0.5、1、1.5、2 倍
delayFactors = [0.5 1 1.5 2];
fig = figure('Name', 'exp2_delay_robustness');
hold on;
metricRows = metric_row('Smith_exact_shift_tau_1', 1, yActual, t, stepAmplitude);

for idx = 1:numel(delayFactors)
    factor = delayFactors(idx);
    Tmismatch = smith_mismatch_tf(C, G0, tau, tau*factor, padeOrder);
    yMismatch = lsim(Tmismatch, uStep, t);
    plot(t, yMismatch, 'LineWidth', 1.4);
    metricRows(end+1) = metric_row(sprintf('Smith_delay_factor_%g', factor), factor, yMismatch, t, stepAmplitude); %#ok<SAGROW>
end
legend(compose('tau factor=%g', delayFactors), 'Location', 'southeast');
common_plot_style(gca, '实验二 Smith 预估器延迟误差鲁棒性', '时间 (s)', '输出');
save_figure(fig, figureDir, 'exp2_delay_robustness.png');

%% 5. Bode 图：Smith 等效开环与传统含延迟开环
[gmSmith, pmSmith, wcgSmith, wcpSmith] = margin(Lsmith);
[gmConv, pmConv, wcgConv, wcpConv] = margin(C * GdelayedPade);

fig = figure('Name', 'exp2_bode_smith');
margin(Lsmith);
common_plot_style(fig);
save_figure(fig, figureDir, 'exp2_bode_smith.png');

fig = figure('Name', 'exp2_bode_conventional');
margin(C * GdelayedPade);
common_plot_style(fig);
save_figure(fig, figureDir, 'exp2_bode_conventional.png');

%% 6. 指标表输出
metricRows(end+1) = metric_row('Conventional_PI_pade_tau_1', 1, yConventional, t, stepAmplitude);
metricsTable = struct2table(metricRows);
metricsTable.NominalTau_s = tau * ones(height(metricsTable), 1);
metricsTable.PlantTimeConstant_s = plantTimeConstant * ones(height(metricsTable), 1);
metricsTable.Kp = Kp * ones(height(metricsTable), 1);
metricsTable.Ki = Ki * ones(height(metricsTable), 1);
metricsTable.SmithPhaseMargin_deg = pmSmith * ones(height(metricsTable), 1);
metricsTable.SmithGainMargin_dB = to_db(gmSmith) * ones(height(metricsTable), 1);
metricsTable.SmithGainCrossover_rad_s = wcpSmith * ones(height(metricsTable), 1);
metricsTable.SmithPhaseCrossover_rad_s = wcgSmith * ones(height(metricsTable), 1);
metricsTable.ConventionalPhaseMargin_deg = pmConv * ones(height(metricsTable), 1);
metricsTable.ConventionalGainMargin_dB = to_db(gmConv) * ones(height(metricsTable), 1);
metricsTable.ConventionalGainCrossover_rad_s = wcpConv * ones(height(metricsTable), 1);
metricsTable.ConventionalPhaseCrossover_rad_s = wcgConv * ones(height(metricsTable), 1);
write_table_safely(metricsTable, tableDir, 'exp2_metrics.csv');

fprintf('实验二完成：图像与指标表已写出。\n');

%% 本脚本局部工具函数
function ensure_output_dirs(figureDir, tableDir)
if exist(figureDir, 'dir') ~= 7
    mkdir(figureDir);
end
if exist(tableDir, 'dir') ~= 7
    mkdir(tableDir);
end
end

function delayed = apply_transport_delay(t, y, delaySeconds)
% 使用插值实现纯迟后，避免依赖特殊延迟绘图接口。
shiftedTime = t - delaySeconds;
delayed = interp1(t, y, shiftedTime, 'linear', 0);
delayed = delayed(:);
end

function sys = smith_mismatch_tf(C, G0, nominalDelay, actualDelay, padeOrder)
% Smith 预估器延迟失配闭环：
% Y/R = C*G0*D_actual / [1 + C*(G0*D_actual + G0 - G0*D_nominal)]
[nomNum, nomDen] = pade(nominalDelay, padeOrder);
[actNum, actDen] = pade(actualDelay, padeOrder);
Dnominal = tf(nomNum, nomDen);
Dactual = tf(actNum, actDen);
denominatorBlock = 1 + C * (G0*Dactual + G0 - G0*Dnominal);
sys = minreal((C * G0 * Dactual) / denominatorBlock, 1e-7);
end

function row = metric_row(labelText, delayFactor, y, t, targetValue)
m = compute_step_metrics(y, t, targetValue, labelText);
row = struct();
row.Method = char(labelText);
row.DelayFactor = delayFactor;
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
    warning('exp2:WriteTableFailed', '写表失败: %s', ME.message);
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
