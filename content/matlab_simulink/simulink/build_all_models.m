function results = build_all_models()
%BUILD_ALL_MODELS 构建三个计算机控制系统实验的 Simulink 模型。
% 运行本文件将创建 outputs/models 目录，并导出适合报告打印的模型截图。

scriptDir = fileparts(mfilename('fullpath'));
packageDir = fileparts(scriptDir);
outputDir = fullfile(packageDir, 'outputs', 'models');
figureDir = fullfile(packageDir, 'outputs', 'figures');

ensure_dir(outputDir, '模型输出目录');
ensure_dir(figureDir, '图像输出目录');

builders = {
    '实验一 数字 PID 参数整定', 'build_exp1_digital_pid_model', 'exp1_digital_pid', 'exp1_simulink_model.png';
    '实验二 Smith 预估控制', 'build_exp2_smith_predictor_model', 'exp2_smith_predictor', 'exp2_simulink_model.png';
    '实验三 二阶对象数字控制', 'build_exp3_discrete_second_order_model', 'exp3_discrete_second_order', 'exp3_simulink_model.png'
};

results = struct('name', {}, 'status', {}, 'modelPath', {}, 'message', {});
oldPath = path;
cleanupObj = onCleanup(@() path(oldPath)); %#ok<NASGU>
addpath(scriptDir);

for k = 1:size(builders, 1)
    experimentName = builders{k, 1};
    builderName = builders{k, 2};
    modelName = builders{k, 3};
    screenshotName = builders{k, 4};
    fprintf('\n开始构建：%s\n', experimentName);

    item = struct('name', experimentName, 'status', '', 'modelPath', '', 'message', '');
    try
        modelPath = feval(builderName, outputDir);
        item.modelPath = modelPath;
        if isempty(modelPath)
            item.status = 'skipped';
            item.message = 'Simulink 不可用，已跳过。';
            fprintf('已跳过：%s（Simulink 不可用）\n', experimentName);
        else
            item.status = 'success';
            item.message = '构建成功。';
            fprintf('构建完成：%s\n', modelPath);
            export_model_screenshot(modelPath, modelName, fullfile(figureDir, screenshotName));
        end
    catch ME
        item.status = 'failed';
        item.message = ME.message;
        fprintf(2, '构建失败：%s\n原因：%s\n', experimentName, ME.message);
    end
    results(end + 1) = item; %#ok<AGROW>
end

fprintf('\n全部构建任务已处理。输出目录：%s\n', outputDir);
end

function ensure_dir(targetDir, labelText)
if ~exist(targetDir, 'dir')
    mkdir(targetDir);
    fprintf('已创建%s：%s\n', labelText, targetDir);
end
end

function export_model_screenshot(modelPath, modelName, screenshotPath)
try
    if exist(modelPath, 'file') ~= 2
        fprintf('跳过模型截图导出：未找到模型文件 %s\n', modelPath);
        return;
    end

    load_system(modelPath);
    open_system(modelName);
    set_if_supported(modelName, 'ScreenColor', 'white');
    set_if_supported(modelName, 'ZoomFactor', 'FitSystem');
    drawnow;
    print(['-s' modelName], '-dpng', '-r600', screenshotPath);
    enhance_simulink_screenshot(screenshotPath);
    close_system(modelName, 0);
    fprintf('模型截图已保存并增强线条：%s\n', screenshotPath);
catch ME
    fprintf(2, '模型截图导出失败：%s\n原因：%s\n', modelName, ME.message);
end
end

function enhance_simulink_screenshot(imagePath)
try
    imageData = imread(imagePath);
    if size(imageData, 3) ~= 3
        return;
    end

    imageDouble = double(imageData);
    luminance = 0.299 * imageDouble(:, :, 1) + 0.587 * imageDouble(:, :, 2) + 0.114 * imageDouble(:, :, 3);
    inkMask = luminance < 185;
    thickMask = false(size(inkMask));

    for rowShift = -1:1
        for colShift = -1:1
            thickMask = thickMask | shift_mask(inkMask, rowShift, colShift);
        end
    end

    for channelIdx = 1:3
        channelData = imageData(:, :, channelIdx);
        channelData(thickMask) = min(channelData(thickMask), uint8(35));
        imageData(:, :, channelIdx) = channelData;
    end

    imwrite(imageData, imagePath);
catch ME
    fprintf(2, '截图线条增强失败：%s\n原因：%s\n', imagePath, ME.message);
end
end

function shifted = shift_mask(mask, rowShift, colShift)
shifted = false(size(mask));
sourceRows = max(1, 1 + rowShift):min(size(mask, 1), size(mask, 1) + rowShift);
sourceCols = max(1, 1 + colShift):min(size(mask, 2), size(mask, 2) + colShift);
targetRows = sourceRows - rowShift;
targetCols = sourceCols - colShift;
shifted(targetRows, targetCols) = mask(sourceRows, sourceCols);
end

function set_if_supported(target, paramName, value)
try
    params = get_param(target, 'ObjectParameters');
    if isfield(params, paramName)
        set_param(target, paramName, value);
    end
catch ME
    fprintf(2, '跳过样式参数 %s：%s\n', paramName, ME.message);
end
end
