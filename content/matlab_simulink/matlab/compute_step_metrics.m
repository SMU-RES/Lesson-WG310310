function metrics = compute_step_metrics(responseOrSystem, timeVector, targetValue, labelText)
%COMPUTE_STEP_METRICS 计算阶跃/跟踪响应的常用动态性能指标。
%   metrics = compute_step_metrics(sys, t, 1, 'PID') 对 LTI 系统进行阶跃分析。
%   metrics = compute_step_metrics(y, t, r, 'PID') 对已经仿真的输出向量 y 进行分析。
%
% 中文说明：指导书要求记录运行结果并分析动态、静态指标。此函数优先调用
% Control System Toolbox 的 stepinfo；若响应异常或 stepinfo 不适用，则使用
% 基本数组运算给出可写表的备用指标，避免脚本硬失败。

if nargin < 2
    timeVector = [];
end
if nargin < 3
    targetValue = [];
end
if nargin < 4 || isempty(labelText)
    labelText = 'response';
end

y = [];
t = [];
info = struct();

try
    if isnumeric(responseOrSystem)
        y = responseOrSystem(:);
        if isempty(timeVector)
            t = (0:numel(y)-1).';
        else
            t = timeVector(:);
        end
        if isempty(targetValue)
            targetValue = y(end);
        end
        info = stepinfo(y, t, targetValue);
    else
        if isempty(timeVector)
            [y, t] = step(responseOrSystem);
        else
            [y, t] = step(responseOrSystem, timeVector);
        end
        y = squeeze(y);
        y = y(:);
        t = t(:);
        if isempty(targetValue)
            targetValue = dcgain(responseOrSystem);
        end
        info = stepinfo(y, t, targetValue);
    end
catch ME
    fprintf(2, 'stepinfo 调用失败，改用备用指标算法：%s\n', ME.message);
end

if isempty(y)
    try
        y = responseOrSystem(:);
        t = timeVector(:);
    catch
        y = NaN;
        t = NaN;
    end
end

if isempty(targetValue) || ~isfinite_scalar(targetValue)
    targetValue = y(end);
end

fallback = fallback_step_info(y, t, targetValue);

metrics = struct();
metrics.Label = char(labelText);
metrics.RiseTime = pick_field(info, fallback, 'RiseTime');
metrics.SettlingTime = pick_field(info, fallback, 'SettlingTime');
metrics.SettlingMin = pick_field(info, fallback, 'SettlingMin');
metrics.SettlingMax = pick_field(info, fallback, 'SettlingMax');
metrics.Overshoot = pick_field(info, fallback, 'Overshoot');
metrics.Undershoot = pick_field(info, fallback, 'Undershoot');
metrics.Peak = pick_field(info, fallback, 'Peak');
metrics.PeakTime = pick_field(info, fallback, 'PeakTime');
metrics.SteadyStateValue = y(end);
metrics.SteadyStateError = targetValue - y(end);
end

function value = pick_field(primary, fallback, fieldName)
if isfield(primary, fieldName) && isfinite_scalar(primary.(fieldName))
    value = primary.(fieldName);
else
    value = fallback.(fieldName);
end
end

function tfValue = isfinite_scalar(value)
tfValue = isnumeric(value) && isscalar(value) && isfinite(value);
end

function info = fallback_step_info(y, t, targetValue)
% 基于 2% 稳态误差带的简单指标估计。
y = y(:);
t = t(:);
if isempty(y) || isempty(t) || numel(y) ~= numel(t)
    y = NaN;
    t = NaN;
end

absTarget = max(abs(targetValue), eps);
band = 0.02 * absTarget;
errorAbs = abs(y - targetValue);
lastOutside = find(errorAbs > band, 1, 'last');
if isempty(lastOutside)
    settlingTime = t(1);
elseif lastOutside < numel(t)
    settlingTime = t(lastOutside + 1);
else
    settlingTime = NaN;
end

if targetValue >= 0
    y10 = 0.10 * targetValue;
    y90 = 0.90 * targetValue;
    idx10 = find(y >= y10, 1, 'first');
    idx90 = find(y >= y90, 1, 'first');
    [peakValue, peakIndex] = max(y);
    overshoot = max(0, (peakValue - targetValue) / absTarget * 100);
else
    y10 = 0.10 * targetValue;
    y90 = 0.90 * targetValue;
    idx10 = find(y <= y10, 1, 'first');
    idx90 = find(y <= y90, 1, 'first');
    [peakValue, peakIndex] = min(y);
    overshoot = max(0, (targetValue - peakValue) / absTarget * 100);
end

if ~isempty(idx10) && ~isempty(idx90)
    riseTime = abs(t(idx90) - t(idx10));
else
    riseTime = NaN;
end

info = struct();
info.RiseTime = riseTime;
info.SettlingTime = settlingTime;
info.SettlingMin = min(y);
info.SettlingMax = max(y);
info.Overshoot = overshoot;
info.Undershoot = 0;
info.Peak = peakValue;
info.PeakTime = t(peakIndex);
end
