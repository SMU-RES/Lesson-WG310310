function common_plot_style(target, plotTitle, xText, yText)
%COMMON_PLOT_STYLE 统一设置计算机控制系统实验图形样式。
%   输出用于打印粘贴到实验本，因此统一使用大画布、白底、黑字和较粗线条。

if nargin < 1 || isempty(target)
    target = gca;
end
if nargin < 2
    plotTitle = '';
end
if nargin < 3
    xText = '';
end
if nargin < 4
    yText = '';
end

try
    targetType = get(target, 'Type');
catch
    target = gca;
    targetType = get(target, 'Type');
end

if strcmpi(targetType, 'figure')
    fig = target;
    axesList = findall(fig, 'Type', 'axes');
else
    axesList = target;
    fig = ancestor(target, 'figure');
end

if ~isempty(fig)
    set(fig, ...
        'Color', 'w', ...
        'InvertHardcopy', 'off', ...
        'Units', 'inches', ...
        'Position', [0.5 0.5 11.0 6.8], ...
        'PaperUnits', 'inches', ...
        'PaperPosition', [0 0 11.0 6.8], ...
        'PaperSize', [11.0 6.8]);
end

for k = 1:numel(axesList)
    ax = axesList(k);
    try
        grid(ax, 'on');
        box(ax, 'on');
        set(ax, ...
            'Color', 'w', ...
            'XColor', 'k', ...
            'YColor', 'k', ...
            'ZColor', 'k', ...
            'GridColor', [0.55 0.55 0.55], ...
            'MinorGridColor', [0.75 0.75 0.75], ...
            'FontName', 'Source Han Sans SC', ...
            'FontSize', 14, ...
            'LineWidth', 1.2);
    catch ME
        fprintf(2, '图形坐标轴样式设置跳过：%s\n', ME.message);
    end
end

if numel(axesList) == 1
    ax = axesList(1);
    if ~isempty(plotTitle)
        title(ax, plotTitle, 'FontWeight', 'bold', 'Color', 'k', 'FontSize', 16);
    end
    if ~isempty(xText)
        xlabel(ax, xText, 'Color', 'k', 'FontSize', 14);
    end
    if ~isempty(yText)
        ylabel(ax, yText, 'Color', 'k', 'FontSize', 14);
    end
end

if ~isempty(fig)
    try
        textObjects = findall(fig, 'Type', 'text');
        set(textObjects, 'Color', 'k', 'FontName', 'Source Han Sans SC');
    catch ME
        fprintf(2, '图中文字样式设置跳过：%s\n', ME.message);
    end
    try
        legendObjects = findall(fig, 'Type', 'legend');
        set(legendObjects, 'TextColor', 'k', 'Color', 'w', 'EdgeColor', 'k', 'FontName', 'Source Han Sans SC', 'FontSize', 12);
    catch ME
        fprintf(2, '图例样式设置跳过：%s\n', ME.message);
    end
    try
        lineObjects = findall(fig, 'Type', 'line');
        set(lineObjects, 'LineWidth', 1.6);
    catch ME
        fprintf(2, '曲线线宽设置跳过：%s\n', ME.message);
    end
end
end
