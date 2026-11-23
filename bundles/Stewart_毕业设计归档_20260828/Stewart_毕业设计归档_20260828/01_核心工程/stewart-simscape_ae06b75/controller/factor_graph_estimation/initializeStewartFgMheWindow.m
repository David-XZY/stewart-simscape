function initial = initializeStewartFgMheWindow(window, config, previous)
% initializeStewartFgMheWindow - 从窗口数据和上一窗口估计构造优化初值
arguments
    window struct
    config struct
    previous struct = struct()
end

W = config.windowLength;
initial = struct();
initial.q = pickWindowMatrix(window, 'q', 6, W, config.anchorPose);
initial.qd = pickWindowMatrix(window, 'qd', 6, W, zeros(6, 1));
initial.qdd = pickWindowMatrix(window, 'qdd', 6, W, zeros(6, 1));

initial.bL = pickPrevious(previous, 'bL', zeros(6, 1));
initial.bAtt = pickPrevious(previous, 'bAtt', zeros(3, 1));
initial.ba = pickPrevious(previous, 'ba', zeros(3, 1));
initial.bg = pickPrevious(previous, 'bg', zeros(3, 1));
initial.dm = pickPrevious(previous, 'dm', 0);
initial.dc = pickPrevious(previous, 'dc', zeros(3, 1));
initial.kF = pickPrevious(previous, 'kF', ones(6, 1));
initial.tauF = pickPrevious(previous, 'tauF', 0);
initial.cF = pickPrevious(previous, 'cF', zeros(6, 1));
end

function value = pickWindowMatrix(window, name, rowCount, columnCount, defaultColumn)
if isfield(window, 'initial') && isfield(window.initial, name)
    value = window.initial.(name);
elseif isfield(window, name)
    value = window.(name);
else
    value = repmat(defaultColumn(:), 1, columnCount);
end
if size(value, 1) ~= rowCount
    error('initializeStewartFgMheWindow:InvalidInitialSize', ...
        '%s 初值行数应为 %d。', name, rowCount);
end
if size(value, 2) < columnCount
    value = [value, repmat(value(:, end), 1, columnCount - size(value, 2))];
end
value = value(:, 1:columnCount);
end

function value = pickPrevious(previous, name, defaultValue)
if isfield(previous, 'estimate') && isfield(previous.estimate, name)
    value = previous.estimate.(name);
elseif isfield(previous, name)
    value = previous.(name);
else
    value = defaultValue;
end
value = value(:);
if isscalar(defaultValue)
    value = value(1);
end
end
