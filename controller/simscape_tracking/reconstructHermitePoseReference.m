function reference = reconstructHermitePoseReference(nodeTime, nodePose, nodeVelocity, sampleTime)
% reconstructHermitePoseReference - 由节点 q/qd 重建导数一致的分段三次 Hermite 参考
arguments
    nodeTime double
    nodePose double
    nodeVelocity double
    sampleTime (1, 1) double {mustBePositive, mustBeFinite}
end

nodeTime = nodeTime(:).';
nodeCount = numel(nodeTime);
if nodeCount < 2 || any(~isfinite(nodeTime)) || any(diff(nodeTime) <= 0)
    error('reconstructHermitePoseReference:InvalidTime', ...
        '节点时间必须为至少包含两个元素的严格递增有限序列。');
end
if ~isequal(size(nodePose), [6, nodeCount]) || ...
        ~isequal(size(nodeVelocity), [6, nodeCount]) || ...
        any(~isfinite(nodePose), 'all') || any(~isfinite(nodeVelocity), 'all')
    error('reconstructHermitePoseReference:InvalidState', ...
        '节点 q 和 qd 必须为有限的 6x%d 数组。', nodeCount);
end

time = nodeTime(1):sampleTime:nodeTime(end);
if time(end) < nodeTime(end) - 10 * eps(nodeTime(end))
    time(end + 1) = nodeTime(end);
else
    time(end) = nodeTime(end);
end

pose = zeros(6, numel(time));
velocity = zeros(6, numel(time));
acceleration = zeros(6, numel(time));
for intervalIndex = 1:nodeCount - 1
    if intervalIndex < nodeCount - 1
        denseIndex = find(time >= nodeTime(intervalIndex) & ...
            time < nodeTime(intervalIndex + 1));
    else
        denseIndex = find(time >= nodeTime(intervalIndex) & ...
            time <= nodeTime(intervalIndex + 1));
    end
    intervalDuration = nodeTime(intervalIndex + 1) - nodeTime(intervalIndex);
    normalizedTime = (time(denseIndex) - nodeTime(intervalIndex)) / intervalDuration;

    h00 = 2 * normalizedTime.^3 - 3 * normalizedTime.^2 + 1;
    h10 = normalizedTime.^3 - 2 * normalizedTime.^2 + normalizedTime;
    h01 = -2 * normalizedTime.^3 + 3 * normalizedTime.^2;
    h11 = normalizedTime.^3 - normalizedTime.^2;
    pose(:, denseIndex) = nodePose(:, intervalIndex) * h00 + ...
        intervalDuration * nodeVelocity(:, intervalIndex) * h10 + ...
        nodePose(:, intervalIndex + 1) * h01 + ...
        intervalDuration * nodeVelocity(:, intervalIndex + 1) * h11;

    dh00 = (6 * normalizedTime.^2 - 6 * normalizedTime) / intervalDuration;
    dh10 = 3 * normalizedTime.^2 - 4 * normalizedTime + 1;
    dh01 = (-6 * normalizedTime.^2 + 6 * normalizedTime) / intervalDuration;
    dh11 = 3 * normalizedTime.^2 - 2 * normalizedTime;
    velocity(:, denseIndex) = nodePose(:, intervalIndex) * dh00 + ...
        nodeVelocity(:, intervalIndex) * dh10 + ...
        nodePose(:, intervalIndex + 1) * dh01 + ...
        nodeVelocity(:, intervalIndex + 1) * dh11;

    ddh00 = (12 * normalizedTime - 6) / intervalDuration^2;
    ddh10 = (6 * normalizedTime - 4) / intervalDuration;
    ddh01 = (-12 * normalizedTime + 6) / intervalDuration^2;
    ddh11 = (6 * normalizedTime - 2) / intervalDuration;
    acceleration(:, denseIndex) = nodePose(:, intervalIndex) * ddh00 + ...
        nodeVelocity(:, intervalIndex) * ddh10 + ...
        nodePose(:, intervalIndex + 1) * ddh01 + ...
        nodeVelocity(:, intervalIndex + 1) * ddh11;
end

reference = struct();
reference.t = time;
reference.q = pose;
reference.qd = velocity;
reference.qdd = acceleration;
reference.nodeTime = nodeTime;
reference.sampleTime = sampleTime;
reference.interpolation = 'cubic-hermite';
end
