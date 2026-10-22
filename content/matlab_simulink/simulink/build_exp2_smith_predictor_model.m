function modelPath = build_exp2_smith_predictor_model(outputDir)
%BUILD_EXP2_SMITH_PREDICTOR_MODEL 构建实验二 Smith 预估控制模型。
% 依据《计算机控制系统》实验指导书：PI 参数 Kp=40, Ki=0.022，阶跃输入 100。
% tau=1 按《计算机控制系统》实验指导书设置，避免使用与指导书冲突的 tau=100。

if nargin < 1 || isempty(outputDir)
    outputDir = default_output_dir();
end

modelPath = '';
if ~simulink_available()
    fprintf('跳过实验二模型构建：未检测到 Simulink 许可证或 Simulink 不可用。\n');
    return;
end

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

modelName = 'exp2_smith_predictor';
modelPath = fullfile(outputDir, [modelName '.slx']);
tau = 1;      % tau=1 按实验指导书设置。
plantDen = '[60 1]';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

new_system(modelName);
set_param(modelName, 'StopTime', '800');

add_block('simulink/Sources/Step', [modelName '/Reference_100'], ...
    'Position', [40 140 70 170]);
set_param([modelName '/Reference_100'], 'Time', '0', 'Before', '0', 'After', '100');

add_block('simulink/Math Operations/Sum', [modelName '/Sum_Error'], ...
    'Position', [125 132 155 178]);
set_param([modelName '/Sum_Error'], 'Inputs', '+-');

add_block('simulink/Continuous/PID Controller', [modelName '/PI_Controller'], ...
    'Position', [210 125 305 180]);
set_param([modelName '/PI_Controller'], 'Controller', 'PI', 'P', '40', 'I', '0.022');

add_block('simulink/Discontinuities/Saturation', [modelName '/Saturation'], ...
    'Position', [350 135 385 170]);
set_param([modelName '/Saturation'], 'UpperLimit', '200', 'LowerLimit', '-200');

add_block('simulink/Continuous/Transfer Fcn', [modelName '/Actual_No_Delay_Plant'], ...
    'Position', [440 120 555 180]);
set_param([modelName '/Actual_No_Delay_Plant'], 'Numerator', '[1]', 'Denominator', plantDen);

add_block('simulink/Continuous/Transport Delay', [modelName '/Actual_Transport_Delay'], ...
    'Position', [600 120 700 180]);
set_param([modelName '/Actual_Transport_Delay'], 'DelayTime', num2str(tau));

add_block('simulink/Continuous/Transfer Fcn', [modelName '/Smith_No_Delay_Model'], ...
    'Position', [440 255 555 315]);
set_param([modelName '/Smith_No_Delay_Model'], 'Numerator', '[1]', 'Denominator', plantDen);

add_block('simulink/Continuous/Transport Delay', [modelName '/Smith_Model_Delay'], ...
    'Position', [600 255 700 315]);
set_param([modelName '/Smith_Model_Delay'], 'DelayTime', num2str(tau));

add_block('simulink/Math Operations/Sum', [modelName '/Smith_Feedback_Sum'], ...
    'Position', [745 205 775 275]);
set_param([modelName '/Smith_Feedback_Sum'], 'Inputs', '++-');

add_block('simulink/Signal Routing/Mux', [modelName '/Mux_Scope'], ...
    'Position', [835 95 845 205]);
set_param([modelName '/Mux_Scope'], 'Inputs', '3');

add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
    'Position', [895 125 935 165]);

add_block('simulink/Sinks/Scope', [modelName '/Control_Scope'], ...
    'Position', [440 35 480 75]);

add_line(modelName, 'Reference_100/1', 'Sum_Error/1', 'autorouting', 'on');
add_line(modelName, 'Reference_100/1', 'Mux_Scope/1', 'autorouting', 'on');
add_line(modelName, 'Sum_Error/1', 'PI_Controller/1', 'autorouting', 'on');
add_line(modelName, 'PI_Controller/1', 'Saturation/1', 'autorouting', 'on');
add_line(modelName, 'Saturation/1', 'Actual_No_Delay_Plant/1', 'autorouting', 'on');
add_line(modelName, 'Saturation/1', 'Smith_No_Delay_Model/1', 'autorouting', 'on');
add_line(modelName, 'Saturation/1', 'Control_Scope/1', 'autorouting', 'on');
add_line(modelName, 'Actual_No_Delay_Plant/1', 'Actual_Transport_Delay/1', 'autorouting', 'on');
add_line(modelName, 'Actual_Transport_Delay/1', 'Smith_Feedback_Sum/1', 'autorouting', 'on');
add_line(modelName, 'Actual_Transport_Delay/1', 'Mux_Scope/2', 'autorouting', 'on');
add_line(modelName, 'Smith_No_Delay_Model/1', 'Smith_Feedback_Sum/2', 'autorouting', 'on');
add_line(modelName, 'Smith_No_Delay_Model/1', 'Smith_Model_Delay/1', 'autorouting', 'on');
add_line(modelName, 'Smith_Model_Delay/1', 'Smith_Feedback_Sum/3', 'autorouting', 'on');
add_line(modelName, 'Smith_Feedback_Sum/1', 'Sum_Error/2', 'autorouting', 'on');
add_line(modelName, 'Smith_Feedback_Sum/1', 'Mux_Scope/3', 'autorouting', 'on');
add_line(modelName, 'Mux_Scope/1', 'Scope/1', 'autorouting', 'on');

apply_print_style(modelName);
save_system(modelName, modelPath);
fprintf('实验二 Simulink 模型已保存：%s\n', modelPath);
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
