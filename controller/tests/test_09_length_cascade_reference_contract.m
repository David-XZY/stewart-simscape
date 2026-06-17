function test_09_length_cascade_reference_contract
% test_09_length_cascade_reference_contract - 验证纯 q/qd 长度级联参考契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(controllerRoot, 'simscape_tracking'));

model = buildOptModelCustom();
sample = load(fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat'), 'refs');
refs = rmfield(sample.refs, {'L', 'Fleg'});

[lengthRefs, references] = generateSimscapeLengthCascadeReferences(refs, model);
nodeCount = numel(refs.t);
denseCount = numel(lengthRefs.t);

assert(denseCount > nodeCount);
assert(max(abs(diff(lengthRefs.t) - 0.01)) < 1e-12);
assert(strcmp(lengthRefs.referenceInterpolation, 'cubic-hermite'));
assert(isequal(lengthRefs.nodeTime, refs.t));
assert(isequal(size(lengthRefs.Lref), [6, denseCount]));
assert(isequal(size(lengthRefs.Ldref), [6, denseCount]));
assert(isequal(size(references.rL.Data), [denseCount, 6]));
assert(isequal(size(references.rLd.Data), [denseCount, 6]));
assert(~isfield(references, 'uFF'));
assert(~isfield(lengthRefs, 'Fleg'));

for nodeIndex = [1, ceil(nodeCount / 2), nodeCount]
    denseIndex = find(abs(lengthRefs.t - refs.t(nodeIndex)) < 1e-12, 1);
    kin = sgpIK(refs.q(:, nodeIndex), model);
    jacobian = sgpJacobian(refs.q(:, nodeIndex), model);
    assert(max(abs(lengthRefs.Lref(:, denseIndex) - kin.L)) < 1e-12);
    assert(max(abs(lengthRefs.Ldref(:, denseIndex) - ...
        jacobian.Jq * refs.qd(:, nodeIndex))) < 1e-12);
end

numericLd = zeros(size(lengthRefs.Lref));
for legIndex = 1:6
    numericLd(legIndex, :) = gradient(lengthRefs.Lref(legIndex, :), lengthRefs.t);
end
assert(max(abs(numericLd(:) - lengthRefs.Ldref(:))) < 0.02);
end
