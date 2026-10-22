function run_all()
%% run_all.m
% 计算机控制系统三个实验的 MATLAB 批量分析入口。
% 本入口使用函数式调度，避免各实验脚本开头的 clear 清除批处理路径变量。

clc;

rootDir = fileparts(mfilename('fullpath'));
matlabDir = fullfile(rootDir, 'matlab');
simulinkDir = fullfile(rootDir, 'simulink');
figureDir = fullfile(rootDir, 'outputs', 'figures');
tableDir = fullfile(rootDir, 'outputs', 'tables');
modelDir = fullfile(rootDir, 'outputs', 'models');

% 按提交包约定加入 MATLAB 与 Simulink 子目录。
if exist(matlabDir, 'dir') == 7
    addpath(matlabDir);
else
    warning('run_all:MissingMatlabDir', 'MATLAB 子目录不存在: %s', matlabDir);
end

if exist(simulinkDir, 'dir') == 7
    addpath(simulinkDir);
else
    fprintf('Simulink 子目录不存在，跳过加入路径: %s\n', simulinkDir);
end

ensure_dir(figureDir);
ensure_dir(tableDir);
ensure_dir(modelDir);

run_experiment(rootDir, fullfile('matlab', 'exp1_digital_pid.m'), ...
    '实验一：数字 PID 参数整定');
run_experiment(rootDir, fullfile('matlab', 'exp2_smith_predictor.m'), ...
    '实验二：Smith 预估控制');
run_experiment(rootDir, fullfile('matlab', 'exp3_discrete_second_order_pid.m'), ...
    '实验三：二阶对象数字 PID 控制');

% 若后续提交包中已有 Simulink 构建函数，则在 MATLAB 分析结束后再构建。
if exist('build_all_models', 'file') == 2
    fprintf('检测到 build_all_models，开始构建 Simulink 模型...\n');
    build_all_models;
else
    fprintf('未检测到 build_all_models，跳过 Simulink 构建。\n');
end

fprintf('全部实验分析完成。图像目录: %s\n', figureDir);
fprintf('全部实验分析完成。表格目录: %s\n', tableDir);
fprintf('全部实验分析完成。模型目录: %s\n', modelDir);
end

function run_experiment(rootDir, relativeScriptPath, labelText)
fprintf('开始运行%s...\n', labelText);
run(fullfile(rootDir, relativeScriptPath));
end

function ensure_dir(targetDir)
if exist(targetDir, 'dir') ~= 7
    mkdir(targetDir);
end
end
