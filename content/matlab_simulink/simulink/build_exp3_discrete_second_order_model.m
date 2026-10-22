function modelPath = build_exp3_discrete_second_order_model(outputDir)
%BUILD_EXP3_DISCRETE_SECOND_ORDER_MODEL 构建实验三二阶对象数字控制模型。
% 依据《计算机控制系统》实验指导书：G(z)=(0.003z+0.001)/(z^2-1.94z+0.9418)，Ts=0.01。

if nargin < 1 || isempty(outputDir)
    outputDir = default_output_dir();
end

modelPath = '';
if ~simulink_available()
    fprintf('跳过实验三模型构建：未检测到 Simulink 许可证或 Simulink 不可用。\n');
    return;
end

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

modelName = 'exp3_discrete_second_order';
modelPath = fullfile(outputDir, [modelName '.slx']);
Ts = 0.01;

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

new_system(modelName);
set_param(modelName, 'StopTime', '5', 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(Ts));

add_block('simulink/Sources/Step', [modelName '/Reference_Step'], ...
    'Position', [40 125 70 155]);
set_param([modelName '/Reference_Step'], 'Time', '0', 'Before', '0', 'After', '1', ...
    'SampleTime', num2str(Ts));

add_block('simulink/Sources/Clock', [modelName '/Clock'], ...
    'Position', [40 240 70 270]);

add_block('simulink/Math Operations/Sum', [modelName '/Sum_Error'], ...
    'Position', [125 118 155 162]);
set_param([modelName '/Sum_Error'], 'Inputs', '+-');

add_block('simulink/Signal Routing/Mux', [modelName '/Mux_PID_Input'], ...
    'Position', [205 85 215 215]);
set_param([modelName '/Mux_PID_Input'], 'Inputs', '3');

add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Digital_PID'], ...
    'Position', [270 115 390 180]);
set_matlab_function_code([modelName '/Digital_PID'], digital_pid_code());

add_block('simulink/Discontinuities/Saturation', [modelName '/Saturation_pm10'], ...
    'Position', [430 128 470 168]);
set_param([modelName '/Saturation_pm10'], 'UpperLimit', '10', 'LowerLimit', '-10');

add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Discrete_Second_Order_Plant'], ...
    'Position', [520 112 665 178]);
set_param([modelName '/Discrete_Second_Order_Plant'], ...
    'Numerator', '[0.003 0.001]', ...
    'Denominator', '[1 -1.94 0.9418]', ...
    'SampleTime', num2str(Ts));

add_block('simulink/Signal Routing/Mux', [modelName '/Mux_Scope'], ...
    'Position', [730 108 740 182]);
set_param([modelName '/Mux_Scope'], 'Inputs', '2');

add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
    'Position', [790 128 830 168]);

add_line(modelName, 'Reference_Step/1', 'Sum_Error/1', 'autorouting', 'on');
add_line(modelName, 'Reference_Step/1', 'Mux_PID_Input/1', 'autorouting', 'on');
add_line(modelName, 'Reference_Step/1', 'Mux_Scope/1', 'autorouting', 'on');
add_line(modelName, 'Sum_Error/1', 'Mux_PID_Input/2', 'autorouting', 'on');
add_line(modelName, 'Clock/1', 'Mux_PID_Input/3', 'autorouting', 'on');
add_line(modelName, 'Mux_PID_Input/1', 'Digital_PID/1', 'autorouting', 'on');
add_line(modelName, 'Digital_PID/1', 'Saturation_pm10/1', 'autorouting', 'on');
add_line(modelName, 'Saturation_pm10/1', 'Discrete_Second_Order_Plant/1', 'autorouting', 'on');
add_line(modelName, 'Discrete_Second_Order_Plant/1', 'Sum_Error/2', 'autorouting', 'on');
add_line(modelName, 'Discrete_Second_Order_Plant/1', 'Mux_Scope/2', 'autorouting', 'on');
add_line(modelName, 'Mux_Scope/1', 'Scope/1', 'autorouting', 'on');

apply_print_style(modelName);
save_system(modelName, modelPath);
fprintf('实验三 Simulink 模型已保存：%s\n', modelPath);
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
    'function y = fcn(u_in)', ...
    '% u_in(1): reference, u_in(2): error, u_in(3): time', ...
    'Ts = 0.01;', ...
    'Kp = 155; Ki = 1.5; Kd = 4.8;', ...
    '', ...
    'persistent err_prev int_prev', ...
    'if isempty(err_prev)', ...
    '    err_prev = 0.0; int_prev = 0.0;', ...
    'end', ...
    '', ...
    'error = u_in(2);', ...
    'int_prev = int_prev + error * Ts;', ...
    'deriv = (error - err_prev) / Ts;', ...
    'y = Kp * error + Ki * int_prev + Kd * deriv;', ...
    'err_prev = error;', ...
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
