function test_20_hermite_reference_contract
% test_20_hermite_reference_contract - 验证 q/qd 分段三次 Hermite 重建契约
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'integration'));

nodeTime = [0, 0.5, 1.0];
nodePose = repmat(nodeTime.^3, 6, 1);
nodeVelocity = repmat(3 * nodeTime.^2, 6, 1);
reference = reconstructHermitePoseReference( ...
    nodeTime, nodePose, nodeVelocity, 0.1);

assert(strcmp(reference.interpolation, 'cubic-hermite'));
assert(isequal(reference.nodeTime, nodeTime));
assert(max(abs(diff(reference.t) - 0.1)) < 1e-12);
assert(max(abs(reference.q(1, :) - reference.t.^3)) < 1e-12);
assert(max(abs(reference.qd(1, :) - 3 * reference.t.^2)) < 1e-12);

for nodeIndex = 1:numel(nodeTime)
    denseIndex = find(abs(reference.t - nodeTime(nodeIndex)) < 1e-12, 1);
    assert(~isempty(denseIndex));
    assert(max(abs(reference.q(:, denseIndex) - nodePose(:, nodeIndex))) < 1e-12);
    assert(max(abs(reference.qd(:, denseIndex) - nodeVelocity(:, nodeIndex))) < 1e-12);
end
end
