function ranking = rankDisturbanceControllers(perCase, config)
% rankDisturbanceControllers - Rank controllers against fixed-LQI matched cases.
arguments
    perCase table
    config struct
end

controllerIds = unique(perCase.controller, 'stable');
baseline = perCase(perCase.controller == "fixed_lqi", :);
if isempty(baseline)
    error('rankDisturbanceControllers:MissingBaseline', ...
        'fixed_lqi rows are required for normalized ranking.');
end

count = numel(controllerIds);
totalScore = inf(count, 1);
medianNoiseScore = inf(count, 1);
worstNoiseScore = inf(count, 1);
improvement = -inf(count, 1);
maximumCombinedRegression = inf(count, 1);
eligible = false(count, 1);
controllerP95 = nan(count, 1);
caseCount = zeros(count, 1);
for controllerIndex = 1:count
    id = controllerIds(controllerIndex);
    rows = perCase(perCase.controller == id, :);
    [common, rowIndex, baseIndex] = intersect(rows.case_id, baseline.case_id, 'stable');
    if numel(common) ~= height(rows)
        continue;
    end
    rmsRatio = rows.equivalent_pose_rms(rowIndex)./ ...
        max(baseline.equivalent_pose_rms(baseIndex), eps);
    peakRatio = rows.equivalent_pose_peak(rowIndex)./ ...
        max(baseline.equivalent_pose_peak(baseIndex), eps);
    weightedRatio = 0.70*rmsRatio+0.30*peakRatio;
    deterministic = rows.noise_seed(rowIndex);
    deterministic = isnan(deterministic);
    deterministicScore = mean(weightedRatio(deterministic));
    noiseRows = rows(rowIndex(~deterministic), :);
    noiseRatios = weightedRatio(~deterministic);
    [noiseMedian, noiseWorst] = collapseNoise(noiseRows, noiseRatios);
    if isnan(noiseMedian)
        totalScore(controllerIndex) = deterministicScore;
    else
        totalScore(controllerIndex) = mean([deterministicScore, noiseMedian]);
    end
    medianNoiseScore(controllerIndex) = noiseMedian;
    worstNoiseScore(controllerIndex) = noiseWorst;
    improvement(controllerIndex) = 1-totalScore(controllerIndex);
    combined = startsWith(rows.disturbance(rowIndex), "wrench_") & ...
        rows.scale(rowIndex) >= 1.0;
    if any(combined)
        maximumCombinedRegression(controllerIndex) = max(weightedRatio(combined)-1);
    else
        maximumCombinedRegression(controllerIndex) = 0;
    end
    eligible(controllerIndex) = all(logical(rows.eligible(rowIndex)));
    controllerP95(controllerIndex) = max(rows.controller_time_p95_s(rowIndex));
    caseCount(controllerIndex) = numel(common);
end

betterThanExisting = improvement >= config.minimumImprovement & ...
    maximumCombinedRegression <= config.maximumCombinedRegression & eligible & ...
    controllerP95 <= config.onlineP95Limit;
ranking = table(controllerIds, totalScore, improvement, medianNoiseScore, ...
    worstNoiseScore, maximumCombinedRegression, controllerP95, eligible, ...
    betterThanExisting, caseCount, 'VariableNames', {'controller', ...
    'weighted_total_error_ratio', 'improvement_fraction', ...
    'noise_median_error_ratio', 'noise_worst_error_ratio', ...
    'maximum_combined_regression', 'controller_p95_s', 'eligible', ...
    'better_than_existing', 'case_count'});
ranking = sortrows(ranking, {'eligible', 'weighted_total_error_ratio'}, ...
    {'descend', 'ascend'});
ranking.rank = (1:height(ranking)).';
ranking = movevars(ranking, 'rank', 'Before', 1);
end

function [medianScore, worstScore] = collapseNoise(rows, ratios)
if isempty(rows)
    medianScore = NaN;
    worstScore = NaN;
    return;
end
keys = rows.disturbance+"|"+compose('%.6g', rows.scale);
groups = unique(keys, 'stable');
medians = zeros(numel(groups), 1);
worsts = zeros(numel(groups), 1);
for index = 1:numel(groups)
    values = ratios(keys == groups(index));
    medians(index) = median(values);
    worsts(index) = max(values);
end
medianScore = mean(medians);
worstScore = max(worsts);
end
