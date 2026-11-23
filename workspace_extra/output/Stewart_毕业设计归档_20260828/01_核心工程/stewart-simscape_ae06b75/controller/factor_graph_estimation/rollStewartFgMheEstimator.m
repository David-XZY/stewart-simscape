function series = rollStewartFgMheEstimator(windows, config)
% rollStewartFgMheEstimator - 在线滚动运行 SC-FG-MHE 窗口估计
arguments
    windows cell
    config struct
end

windowCount = numel(windows);
results = cell(1, windowCount);
previous = struct();
for index = 1:windowCount
    initial = initializeStewartFgMheWindow(windows{index}, config, previous);
    results{index} = solveStewartFgMheWindow(windows{index}, initial, config, previous);
    previous = results{index};
end

series = struct();
series.windows = windows;
series.results = results;
series.finalEstimate = results{end}.estimate;
series.failureCount = sum(cellfun(@(item) ~item.success, results));
series.solveTimes = cellfun(@(item) item.solveTime, results);
end
