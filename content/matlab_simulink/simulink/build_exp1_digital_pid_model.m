function modelPath = build_exp1_digital_pid_model(outputDir)
%BUILD_EXP1_DIGITAL_PID_MODEL 构建实验一“数字 PID 参数整定”模型。
% 依据《计算机控制系统》实验指导书：被控对象 G(s)=133/(s^2+25s)。
% 参考报告组织方式：参考输入、负反馈、MATLAB Function 数字 PID、零阶保持、示波器。

if nargin < 1 || isempty(outputDir)
    outputDir = default_output_dir();
end

modelPath = '';
if ~simulink_available()
    fprintf('跳过实验一模型构建：未检测到 Simulink 许可证或 Simulink 不可用。\n');
    return;
end

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

modelName = 'exp1_digital_pid';
modelPath = fullfile(outputDir, [modelName '.slx']);

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

new_system(modelName);
set_param(modelName, 'StopTime', '8');

add_block('simulink/Sources/Step', [modelName '/Reference'], ...
    'Position', [40 135 70 165]);
set_param([modelName '/Reference'], 'Time', '0', 'Before', '0', 'After', '1');

add_block('simulink/Sources/Clock', [modelName '/Clock'], ...
    'Position', [40 55 70 85]);

add_block('simulink/Math Operations/Sum', [modelName '/Sum_Error'], ...
    'Position', [125 128 155 172]);
set_param([modelName '/Sum_Error'], 'Inputs', '+-');

add_block('simulink/Signal Routing/Mux', [modelName '/Mux_PID_Input'], ...
    'Position', [205 80 215 180]);
set_param([modelName '/Mux_PID_Input'], 'Inputs', '2');

add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Digital_PID'], ...
    'Position', [265 105 385 175]);
set_matlab_function_code([modelName '/Digital_PID'], digital_pid_code());

add_block('simulink/Discrete/Zero-Order Hold', [modelName '/Zero_Order_Hold'], ...
    'Position', [425 120 485 160]);
set_param([modelName '/Zero_Order_Hold'], 'SampleTime', '0.01');

add_block('simulink/Continuous/Transfer Fcn', [modelName '/Plant_133_over_s2_25s'], ...
    'Position', [535 108 655 172]);
set_param([modelName '/Plant_133_over_s2_25s'], ...
    'Numerator', '[133]', 'Denominator', '[1 25 0]');

add_block('simulink/Signal Routing/Mux', [modelName '/Mux_Scope'], ...
    'Position', [715 105 725 180]);
set_param([modelName '/Mux_Scope'], 'Inputs', '2');

add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
    'Position', [775 125 815 165]);

add_line(modelName, 'Clock/1', 'Mux_PID_Input/1', 'autorouting', 'on');
add_line(modelName, 'Reference/1', 'Sum_Error/1', 'autorouting', 'on');
add_line(modelName, 'Sum_Error/1', 'Mux_PID_Input/2', 'autorouting', 'on');
add_line(modelName, 'Mux_PID_Input/1', 'Digital_PID/1', 'autorouting', 'on');
add_line(modelName, 'Digital_PID/1', 'Zero_Order_Hold/1', 'autorouting', 'on');
add_line(modelName, 'Zero_Order_Hold/1', 'Plant_133_over_s2_25s/1', 'autorouting', 'on');
add_line(modelName, 'Plant_133_over_s2_25s/1', 'Sum_Error/2', 'autorouting', 'on');
add_line(modelName, 'Reference/1', 'Mux_Scope/1', 'autorouting', 'on');
add_line(modelName, 'Plant_133_over_s2_25s/1', 'Mux_Scope/2', 'autorouting', 'on');
add_line(modelName, 'Mux_Scope/1', 'Scope/1', 'autorouting', 'on');

apply_print_style(modelName);
save_system(modelName, modelPath);
fprintf('实验一 Simulink 模型已保存：%s\n', modelPath);
end

function outputDir = default_output_dir()
scriptDir = fileparts(mfilename('fullpath'));
packageDir = fileparts(scriptDir);
outputDir = fullfile(packageDir, 'outputs', 'models');
end

function tf = simulink_available()
try
    tf = license('test', 'Simulink') && ~isempty(which('new_system'));
catch
    tf = false;
end
end

function codeText = digital_pid_code()
codeText = strjoin({ ...
    'function u_out = fcn(u_in)', ...
    '% u_in(1): time, u_in(2): error', ...
    'Kp = 15; Ki = 2; Kd = 0.5;', ...
    '', ...
    'persistent err_prev err_sum t_prev', ...
    'if isempty(err_prev)', ...
    '    err_prev = 0; err_sum = 0; t_prev = 0;', ...
    'end', ...
    '', ...
    't_curr = u_in(1);', ...
    'err_curr = u_in(2);', ...
    'dt = t_curr - t_prev;', ...
    '', ...
    'if dt > 0', ...
    '    p_part = Kp * err_curr;', ...
    '    err_sum = err_sum + err_curr * dt;', ...
    '    i_part = Ki * err_sum;', ...
    '    d_part = Kd * (err_curr - err_prev) / dt;', ...
    'else', ...
    '    p_part = Kp * err_curr;', ...
    '    i_part = 0;', ...
    '    d_part = 0;', ...
    'end', ...
    '', ...
    'u_out = p_part + i_part + d_part;', ...
    'err_prev = err_curr;', ...
    't_prev = t_curr;', ...
    'end' ...
}, newline);
end

function set_matlab_function_code(blockPath, codeText)
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('未找到 MATLAB Function 模块：%s', blockPath);
end
chart(1).Script = codeText;
end

function apply_print_style(modelName)
set_param(modelName, 'ScreenColor', 'white');
blocks = find_system(modelName, 'LookUnderMasks', 'all', 'Type', 'Block');
for idx = 1:numel(blocks)
    set_if_supported(blocks{idx}, 'ForegroundColor', 'black');
    set_if_supported(blocks{idx}, 'BackgroundColor', 'white');
    set_if_supported(blocks{idx}, 'FontSize', '14');
    set_if_supported(blocks{idx}, 'FontWeight', 'bold');
end

lineHandles = find_system(modelName, 'FindAll', 'on', 'Type', 'line');
for idx = 1:numel(lineHandles)
    set_if_supported(lineHandles(idx), 'ForegroundColor', 'black');
    set_if_supported(lineHandles(idx), 'LineWidth', '4');
    set_if_supported(lineHandles(idx), 'FontSize', '13');
end
end

function set_if_supported(target, paramName, value)
params = get_param(target, 'ObjectParameters');
if isfield(params, paramName)
    set_param(target, paramName, value);
end
end
