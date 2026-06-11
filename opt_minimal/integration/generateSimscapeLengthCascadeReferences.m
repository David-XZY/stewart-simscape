function [lengthRefs, references] = generateSimscapeLengthCascadeReferences(refs, model)
% generateSimscapeLengthCascadeReferences - 仅由 q/qd 生成长度级联参考
arguments
    refs struct
    model struct
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

Lref = zeros(6, nodeCount);
Ldref = zeros(6, nodeCount);
for nodeIndex = 1:nodeCount
    kin = sgpIK(refs.q(:, nodeIndex), model);
    jacobian = sgpJacobian(refs.q(:, nodeIndex), model);
    Lref(:, nodeIndex) = kin.L;
    Ldref(:, nodeIndex) = jacobian.Jq * refs.qd(:, nodeIndex);
end

lengthRefs = struct();
lengthRefs.t = time.';
lengthRefs.q = refs.q;
lengthRefs.qd = refs.qd;
lengthRefs.q0 = refs.q(:, 1);
lengthRefs.Lref = Lref;
lengthRefs.Ldref = Ldref;

references = struct();
references.r = timeseries((refs.q - refs.q(:, 1)).', time);
references.rL = timeseries((Lref - Lref(:, 1)).', time);
references.rLd = timeseries(Ldref.', time);
references.description = '仅由 q/qd 生成的纯长度串级控制参考轨迹';
end
