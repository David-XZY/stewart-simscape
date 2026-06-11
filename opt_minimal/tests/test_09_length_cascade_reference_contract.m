function test_09_length_cascade_reference_contract
% test_09_length_cascade_reference_contract - 验证纯 q/qd 长度级联参考契约
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'core'));
addpath(fullfile(optRoot, 'integration'));

model = buildOptModelCustom();
sample = load(fullfile(optRoot, 'examples', 'ihsid_40x20_limited_memory', ...
    'simscape_references.mat'), 'refs');
refs = rmfield(sample.refs, {'L', 'Fleg'});

[lengthRefs, references] = generateSimscapeLengthCascadeReferences(refs, model);
nodeCount = numel(refs.t);

assert(isequal(size(lengthRefs.Lref), [6, nodeCount]));
assert(isequal(size(lengthRefs.Ldref), [6, nodeCount]));
assert(isequal(size(references.rL.Data), [nodeCount, 6]));
assert(isequal(size(references.rLd.Data), [nodeCount, 6]));
assert(~isfield(references, 'uFF'));
assert(~isfield(lengthRefs, 'Fleg'));

for nodeIndex = [1, ceil(nodeCount / 2), nodeCount]
    kin = sgpIK(refs.q(:, nodeIndex), model);
    jacobian = sgpJacobian(refs.q(:, nodeIndex), model);
    assert(max(abs(lengthRefs.Lref(:, nodeIndex) - kin.L)) < 1e-12);
    assert(max(abs(lengthRefs.Ldref(:, nodeIndex) - ...
        jacobian.Jq * refs.qd(:, nodeIndex))) < 1e-12);
end

numericLd = zeros(size(lengthRefs.Lref));
for legIndex = 1:6
    numericLd(legIndex, :) = gradient(lengthRefs.Lref(legIndex, :), refs.t);
end
assert(max(abs(numericLd(:) - lengthRefs.Ldref(:))) < 0.08);
end
