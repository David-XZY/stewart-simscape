function test_86_strict_filter_tightening_contract
% Candidate strict-QP path must inherit the current 0.5 mm hold margin.
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot, 'src'));
addpath(fullfile(projectRoot, 'opt_minimal', 'core'));
addpath(fullfile(projectRoot, 'controller', 'strict_clf_cbf_qp'));
addpath(fullfile(projectRoot, 'controller', 'disturbance_control_comparison'));
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
config = makeDisturbanceComparisonConfig();
[filterScene, strictConfig] = makeDisturbanceStrictFilter(model, scene, config);
margin = 5e-4;
assert(abs(filterScene.collision.safeDistance-scene.collision.safeDistance-margin) < 1e-12);
assert(abs(strictConfig.collision.roofStage1Distance- ...
    scene.collision.stage1ConstraintDistance-margin) < 1e-12);
assert(abs(strictConfig.collision.roofFinalDistance-scene.collision.finalGap-margin) < 1e-12);
assert(abs(strictConfig.collision.sideDistance-scene.collision.safeDistance-margin) < 1e-12);
fprintf('test_86_strict_filter_tightening_contract passed.\n');
end
