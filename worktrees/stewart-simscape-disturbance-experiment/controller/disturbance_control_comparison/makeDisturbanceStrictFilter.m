function [filterScene, strictConfig] = makeDisturbanceStrictFilter(model, physicalScene, config)
% makeDisturbanceStrictFilter - Match the current Simscape strict-QP tightening.
sampleHoldMargin = 5e-4;
filterScene = physicalScene;
filterScene.collision.safeDistance = physicalScene.collision.safeDistance+sampleHoldMargin;
filterScene.collision.stage1ConstraintDistance = ...
    physicalScene.collision.stage1ConstraintDistance+sampleHoldMargin;
filterScene.collision.finalGap = physicalScene.collision.finalGap+sampleHoldMargin;

overrides = config.strictOverrides;
if ~isfield(overrides, 'cbf'), overrides.cbf = struct(); end
if ~isfield(overrides.cbf, 'collisionAlpha1'), overrides.cbf.collisionAlpha1 = 12; end
if ~isfield(overrides.cbf, 'collisionAlpha2'), overrides.cbf.collisionAlpha2 = 12; end
if ~isfield(overrides, 'collision'), overrides.collision = struct(); end
if ~isfield(overrides.collision, 'roofStage1Distance')
    overrides.collision.roofStage1Distance = ...
        physicalScene.collision.stage1ConstraintDistance+sampleHoldMargin;
end
if ~isfield(overrides.collision, 'roofFinalDistance')
    overrides.collision.roofFinalDistance = physicalScene.collision.finalGap+sampleHoldMargin;
end
if ~isfield(overrides.collision, 'sideDistance')
    overrides.collision.sideDistance = physicalScene.collision.safeDistance+sampleHoldMargin;
end
overrides.dt = config.sampleTime;
strictConfig = makeStrictClfCbfQpConfig(model, overrides);
end
