function test_02_simscape_export_contract
% test_02_simscape_export_contract - 验证 Simscape timeseries 导出契约
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'integration'));

nodeCount = 5;
traj = struct();
traj.t = linspace(0, 1, nodeCount);
traj.Q = reshape(1:(6*nodeCount), 6, nodeCount);
traj.V = zeros(6, nodeCount);
traj.Unode = ones(6, nodeCount);
traj.L = 0.5 * ones(6, nodeCount);
scene = struct('qWaypoint', zeros(6, 1), 'qGoal', ones(6, 1), 'q0', -ones(6, 1));
disc = struct('durationApproach', 0.6, 'durationInsertion', 0.4, 'waypointNodeIndex', 4);

[refs, references] = exportTrajectoryToSimscape(traj, scene, disc);
assert(isequal(size(references.r.Data), [nodeCount, 6]));
assert(isequal(size(references.rL.Data), [nodeCount, 6]));
assert(isequal(references.r.Time, traj.t(:)));
assert(isequal(references.rL.Time, traj.t(:)));
assert(isequal(refs.q, traj.Q));
assert(isequal(refs.qd, traj.V));
assert(isequal(refs.Fleg, traj.Unode));
assert(isequal(refs.L, traj.L));
end
