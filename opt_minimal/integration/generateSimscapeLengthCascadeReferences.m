function [lengthRefs, references] = generateSimscapeLengthCascadeReferences(refs, model, sampleTime)
% generateSimscapeLengthCascadeReferences - 仅由 q/qd 生成长度级联参考
arguments
    refs struct
    model struct
    sampleTime (1, 1) double {mustBePositive, mustBeFinite} = 0.01
end

requiredFields = {'t', 'q', 'qd'};
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(refs, requiredFields{fieldIndex})
        error('generateSimscapeLengthCascadeReferences:InvalidReferences', ...
            'refs 缺少必需字段 %s。', requiredFields{fieldIndex});
    end
end

time = refs.t(:);
nodeCount = numel(time);
if nodeCount < 2 || any(~isfinite(time)) || any(diff(time) <= 0)
    error('generateSimscapeLengthCascadeReferences:InvalidReferences', ...
        'refs.t 必须为至少包含两个节点的严格递增有限时间序列。');
end
if ~isequal(size(refs.q), [6, nodeCount]) || ...
        ~isequal(size(refs.qd), [6, nodeCount]) || ...
        any(~isfinite(refs.q), 'all') || any(~isfinite(refs.qd), 'all')
    error('generateSimscapeLengthCascadeReferences:InvalidReferences', ...
        'refs.q 和 refs.qd 必须为有限的 6x%d 数组。', nodeCount);
end

poseReference = reconstructHermitePoseReference(time, refs.q, refs.qd, sampleTime);
denseCount = numel(poseReference.t);
Lref = zeros(6, denseCount);
Ldref = zeros(6, denseCount);
for sampleIndex = 1:denseCount
    kin = sgpIK(poseReference.q(:, sampleIndex), model);
    jacobian = sgpJacobian(poseReference.q(:, sampleIndex), model);
    Lref(:, sampleIndex) = kin.L;
    Ldref(:, sampleIndex) = jacobian.Jq * poseReference.qd(:, sampleIndex);
end

lengthRefs = struct();
lengthRefs.t = poseReference.t;
lengthRefs.q = poseReference.q;
lengthRefs.qd = poseReference.qd;
lengthRefs.q0 = refs.q(:, 1);
lengthRefs.Lref = Lref;
lengthRefs.Ldref = Ldref;
lengthRefs.nodeTime = poseReference.nodeTime;
lengthRefs.referenceInterpolation = poseReference.interpolation;

references = struct();
references.r = timeseries((poseReference.q - refs.q(:, 1)).', poseReference.t(:));
references.rL = timeseries((Lref - Lref(:, 1)).', poseReference.t(:));
references.rLd = timeseries(Ldref.', poseReference.t(:));
references.description = '由节点 q/qd 经三次 Hermite 重建的纯长度串级控制参考轨迹';
end
