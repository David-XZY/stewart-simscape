function result = refreshDisturbanceCollisionAssessment(result)
% refreshDisturbanceCollisionAssessment - Recompute time-varying collision gate.
% This function updates metrics only from stored raw trajectories; it does
% not rerun or alter any controller simulation.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(projectRoot, 'controller', 'strict_clf_cbf_qp'));
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
overrides = result.config.strictOverrides;
overrides.dt = result.config.sampleTime;
strictConfig = makeStrictClfCbfQpConfig(model, overrides);

result.exactRuns = refreshRuns(result.exactRuns, scene, strictConfig, result.config);
result.exactPerCase = summarizeDisturbanceRuns(result.exactRuns, "exact_validation");
result.exactRanking = rankDisturbanceControllers(result.exactPerCase, result.config);
if result.simscape.executed
    result.simscape.runs = refreshRuns(result.simscape.runs, scene, ...
        strictConfig, result.config);
    result.simscape.perCase = summarizeDisturbanceRuns( ...
        result.simscape.runs, "simscape_validation");
    result.simscape.ranking = rankDisturbanceControllers( ...
        result.simscape.perCase, result.config);
    result.perCase = result.simscape.perCase;
    result.ranking = result.simscape.ranking;
else
    result.perCase = result.exactPerCase;
    result.ranking = result.exactRanking;
end
result.conclusion = conclusionText(result.ranking, result.simscape.executed);
end

function runs = refreshRuns(runs, scene, strictConfig, config)
for runIndex = 1:numel(runs)
    if ~isfield(runs(runIndex), 'dense') || ...
            ~isfield(runs(runIndex).dense, 'collisionDistance') || ...
            isempty(runs(runIndex).dense.collisionDistance)
        continue;
    end
    time = runs(runIndex).dense.time(:).';
    distance = runs(runIndex).dense.collisionDistance;
    if size(distance, 2) ~= numel(time), distance = distance.'; end
    threshold = repmat([scene.collision.finalGap; scene.collision.safeDistance; ...
        scene.collision.safeDistance], 1, numel(time));
    for index = 1:numel(time)
        roof = evaluateStrictRoofThreshold(time(index), scene, strictConfig);
        threshold(1, index) = roof.value;
    end
    margin = distance-threshold;
    metrics = runs(runIndex).metrics;
    metrics.minCollisionMargin = min(margin, [], 'all');
    metrics.collisionViolationCount = sum(any( ...
        margin < -config.stateConstraintTolerance, 1));
    metrics.hardConstraintsPassed = metrics.lengthViolationCount == 0 && ...
        metrics.speedViolationCount == 0 && metrics.accelerationViolationCount == 0 && ...
        metrics.forceViolationCount == 0 && metrics.forceRateViolationCount == 0 && ...
        metrics.collisionViolationCount == 0 && metrics.singularityViolationCount == 0;
    metrics.eligible = metrics.hardConstraintsPassed && ...
        metrics.fullTrajectoryCompleted && metrics.infeasibleCount == 0 && ...
        metrics.fallbackCount == 0 && metrics.nonfiniteCount == 0 && ...
        metrics.controllerTimeP95 <= config.onlineP95Limit;
    runs(runIndex).metrics = metrics;
end
end

function text = conclusionText(ranking, hasSimscape)
source = "exact nonlinear validation";
if hasSimscape, source = "Simscape"; end
accepted = ranking(ranking.better_than_existing, :);
if isempty(accepted)
    text = "No controller met every 'better than existing control' gate in "+ ...
        source+". The ranked errors remain useful, but no superiority claim is made.";
else
    best = accepted(1, :);
    text = sprintf(['%s is the best accepted controller in %s: aggregate ' ...
        'weighted error improved by %.2f%% with all declared gates passed.'], ...
        best.controller, source, 100*best.improvement_fraction);
end
end
