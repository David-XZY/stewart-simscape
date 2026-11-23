work = 'D:/NPU/Matlab_code/stewart_unified_nmpc';
audit = 'D:/NPU/Matlab_code/cleanup_20261117';
cd(audit);
addpath(work,fullfile(work,'third_party','casadi'));
entry = {'run_unified','run_core_route','build_experiment','generate_dataset','train_diffusion','evaluate_diffusion','infer_diffusion','run_final_acceptance','audit_core_task_loaders','unified_nmpc_sfun','unified_lqi_sfun'};
for k = 1:numel(entry)
    assert(startsWith(replace(which(entry{k}),'\','/'),work),entry{k});
end
m = jsondecode(fileread(fullfile(work,'tasksets/core_v3_20261111/manifest.json')));
for k = 1:numel(m.tasks)
    b = load_core_problem(m.tasks(k).id);
    assert(strcmp(b.task.id,m.tasks(k).id));
end
legacy = sn.config('translation_detour');
assert(isfile(legacy.assets.checkpoint) && isfile(legacy.assets.expert) && isfile(legacy.workspace.artifact));
checkpointVars = whos('-file',legacy.assets.checkpoint);
assert(~isempty(checkpointVars));
for name = {'unified_stewart_plant','unified_experiment'}
    file = fullfile(legacy.models,[name{1} '.slx']);
    load_system(file);
    assert(bdIsLoaded(name{1}));
end
driver = casadi.SX.sym('cleanupSmoke');
fun = casadi.Function('cleanupSmoke', {driver}, {driver^2});
assert(full(fun(3)) == 9);
assert(casadi.has_conic('hpipm'));
bridgeFiles = dir(fullfile(work,'cache','nmpc_rti_20261115','native_qp','**','rti_hpipm_native.dll'));
assert(~isempty(bridgeFiles));
bridge = casadi.external('rti_hpipm_native',fullfile(bridgeFiles(1).folder,bridgeFiles(1).name));
assert(bridge.n_in() > 0);
pythonExe = fullfile(work,'cache','long_payload_v2_20261109','python','Scripts','python.exe');
assert(isfile(pythonExe));
[status,~] = system(sprintf('"%s" --version',pythonExe));
assert(status == 0);
platform = sn.kernel(legacy);
u = full(platform.inverse(legacy.task.x0,zeros(6,1)));
assert(all(isfinite(u)) && numel(u)==6);
run(fullfile(work,'nmpc_simple','test_geometry.m'));
close_system('unified_experiment',0);
close_system('unified_stewart_plant',0);
fprintf('CLEANUP MATLAB SMOKE PASSED: %d v3 tasks, %d entry points, two SLX, checkpoint, CasADi, HPIPM and Python\n',numel(m.tasks),numel(entry));
