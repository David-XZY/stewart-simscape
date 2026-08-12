function paths = exportDisturbanceComparisonEvidence(result, outputDir, evidenceDir, config)
% exportDisturbanceComparisonEvidence - Full local MAT plus tracked light evidence.
arguments
    result struct
    outputDir {mustBeTextScalar}
    evidenceDir {mustBeTextScalar}
    config struct
end
outputDir = char(outputDir);
evidenceDir = char(evidenceDir);
if ~exist(outputDir, 'dir'), mkdir(outputDir); end
if ~exist(evidenceDir, 'dir'), mkdir(evidenceDir); end

paths = struct();
paths.matFile = string(fullfile(outputDir, 'disturbance_control_comparison.mat'));
paths.perCaseCsv = string(fullfile(evidenceDir, 'per_case_metrics.csv'));
paths.summaryCsv = string(fullfile(evidenceDir, 'controller_summary.csv'));
paths.exactSummaryCsv = string(fullfile(evidenceDir, 'exact_controller_summary.csv'));
paths.screeningSummaryCsv = string(fullfile(evidenceDir, 'candidate_screening_summary.csv'));
paths.dobTuningCsv = string(fullfile(evidenceDir, 'dob_tuning.csv'));
paths.configJson = string(fullfile(evidenceDir, 'experiment_config.json'));
paths.conclusion = string(fullfile(evidenceDir, 'conclusion.md'));
writetable(result.perCase, paths.perCaseCsv);
writetable(result.ranking, paths.summaryCsv);
writetable(result.exactRanking, paths.exactSummaryCsv);
writetable(result.screeningRanking, paths.screeningSummaryCsv);
writetable(result.dobTuning, paths.dobTuningCsv);
writeJson(paths.configJson, serializableConfig(config));
writeConclusion(paths.conclusion, result);
if config.exportFigures
    paths.figures = exportFigures(result, evidenceDir);
end
save(paths.matFile, 'result', '-v7.3');
end

function value = serializableConfig(config)
value = config;
value.controllerIds = cellstr(string(value.controllerIds));
value.baselineControllerIds = cellstr(string(value.baselineControllerIds));
value.candidateControllerIds = cellstr(string(value.candidateControllerIds));
end

function writeJson(fileName, value)
fid = fopen(fileName, 'w');
if fid == -1, error('exportDisturbanceComparisonEvidence:OpenFailed', ...
        'Cannot open %s.', fileName); end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(value, 'PrettyPrint', true));
end

function writeConclusion(fileName, result)
fid = fopen(fileName, 'w');
if fid == -1, error('exportDisturbanceComparisonEvidence:OpenFailed', ...
        'Cannot open %s.', fileName); end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Stewart platform disturbance-control comparison\n\n');
fprintf(fid, 'Generated: %s\n\n', string(result.generatedAt));
fprintf(fid, 'Equivalent error: `sqrt(||e_p||^2 + (0.5 ||e_r||)^2)`.\n\n');
fprintf(fid, 'All ranked controllers used the same trajectory, initial state, model, ');
fprintf(fid, 'actuator boundary, 0.01 s sample time, case list, and random seeds.\n\n');
fprintf(fid, '## Ranking\n\n');
fprintf(fid, '|Rank|Controller|Total ratio|Improvement|Eligible|Better than existing|\n');
fprintf(fid, '|---:|---|---:|---:|:---:|:---:|\n');
for index = 1:height(result.ranking)
    row = result.ranking(index, :);
    fprintf(fid, '|%d|%s|%.6g|%.2f%%|%d|%d|\n', row.rank, row.controller, ...
        row.weighted_total_error_ratio, 100*row.improvement_fraction, ...
        row.eligible, row.better_than_existing);
end
fprintf(fid, '\n## Conclusion\n\n%s\n', result.conclusion);
fprintf(fid, '\n## Acceptance diagnostics\n\n');
fprintf(fid, ['|Controller|Completed|Hard-pass cases|Acceleration violation cases|' ...
    'Force-rate violation cases|Collision violation cases|Fallbacks|Infeasible|' ...
    'Worst P95 (ms)|\n']);
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|---:|\n');
controllers = unique(result.perCase.controller, 'stable');
for index = 1:numel(controllers)
    rows = result.perCase(result.perCase.controller == controllers(index), :);
    completed = sum(rows.full_trajectory_completed > 0.5);
    hardPassed = sum(rows.hard_constraints_passed > 0.5);
    accelerationCases = sum(rows.acceleration_violation_count > 0);
    forceRateCases = sum(rows.force_rate_violation_count > 0);
    collisionCases = sum(rows.collision_violation_count > 0);
    fallbackCount = sum(rows.fallback_count);
    infeasibleCount = sum(rows.infeasible_count);
    worstP95 = 1e3*max(rows.controller_time_p95_s);
    fprintf(fid, '|%s|%d/%d|%d/%d|%d|%d|%d|%d|%d|%.4g|\n', ...
        controllers(index), completed, height(rows), hardPassed, height(rows), ...
        accelerationCases, forceRateCases, collisionCases, fallbackCount, ...
        infeasibleCount, worstP95);
end

fprintf(fid, '\n## Engineering conclusion and next optimization boundary\n\n');
fprintf(fid, ['- `scheduled_lqi_dob` is the tracking winner, but it is not an ' ...
    'accepted controller: high-intensity target noise causes leg-acceleration ' ...
    'violations, some cases exceed the force-rate boundary, and the physical ' ...
    'collision margin remains negative.\n']);
fprintf(fid, ['- `scheduled_lqi_dob_strict_qp` removes the force-rate issue and ' ...
    'greatly reduces total tracking error, but it still has acceleration/' ...
    'collision violations, QP infeasible/fallback events, and a worst-case ' ...
    'P95 above 10 ms.\n']);
fprintf(fid, ['- The current `strict_qp` is not robust to pose-command noise in ' ...
    'this matrix; fallback-heavy cases dominate its aggregate error. It should ' ...
    'not be described as a tracking improvement over fixed LQI.\n']);
fprintf(fid, ['- The DOB-aware strict path models the paired leg-force ' ...
    'compensation and estimated external wrench in acceleration-level CLF/CBF ' ...
    'predictions. Remaining optimization should add a target-command prefilter/' ...
    'command governor with feedforward-consistent derivatives and reduce QP ' ...
    'execution cost before retuning tracking weights.\n']);

fprintf(fid, '\n## Appendix boundary\n\n');
fprintf(fid, ['Ideal leg-length PIDF and pose-to-leg-length cascade use a different ' ...
    'actuator/control boundary. They remain appendix references and are not ' ...
    'included in the total ranking.\n']);
fprintf(fid, '\n## Evidence boundary\n\n');
fprintf(fid, ['A configuration is called better only if it reduces the aggregate ' ...
    'weighted error by at least 15%%, has no medium/high combined-case regression ' ...
    'above 10%%, passes all hard constraints without fallback/nonfinite values, ' ...
    'and keeps online P95 at or below 10 ms.\n']);
end

function paths = exportFigures(result, outputDir)
paths = strings(2, 1);
ranking = result.ranking;
figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1100, 520]);
bar(categorical(ranking.controller), ranking.weighted_total_error_ratio);
yline(0.85, 'r--', '15% improvement threshold');
grid on; ylabel('weighted total error / fixed LQI');
title('Disturbance-control aggregate comparison');
paths(1) = string(fullfile(outputDir, 'aggregate_error_ranking.png'));
exportgraphics(figureHandle, paths(1), 'Resolution', 180);
close(figureHandle);

figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1100, 520]);
hold on;
controllers = unique(result.perCase.controller, 'stable');
for index = 1:numel(controllers)
    rows = result.perCase(result.perCase.controller == controllers(index), :);
    scatter(rows.equivalent_pose_rms, rows.equivalent_pose_peak, 24, ...
        'filled', 'DisplayName', controllers(index));
end
grid on; xlabel('equivalent pose RMS'); ylabel('equivalent pose peak');
title('All matched disturbance cases'); legend('Location', 'best');
paths(2) = string(fullfile(outputDir, 'rms_peak_scatter.png'));
exportgraphics(figureHandle, paths(2), 'Resolution', 180);
close(figureHandle);
end
