function [output, layout] = buildStewartFgMheVariables(action, input, config, layout)
% buildStewartFgMheVariables - SC-FG-MHE 结构体变量和优化向量互转
arguments
    action {mustBeTextScalar}
    input
    config struct
    layout struct = struct()
end

action = string(action);
if action == "pack"
    layout = makeLayout(config);
    output = packVariables(input, layout);
elseif action == "unpack"
    if ~isfield(layout, 'fields')
        error('buildStewartFgMheVariables:MissingLayout', '解包时必须传入 layout。');
    end
    output = unpackVariables(input, layout);
else
    error('buildStewartFgMheVariables:UnknownAction', '未知变量打包动作：%s。', action);
end
end

function layout = makeLayout(config)
W = config.windowLength;
fields = {
    'q', [6, W]
    'qd', [6, W]
    'qdd', [6, W]
    'bL', [6, 1]
    'bAtt', [3, 1]
    'ba', [3, 1]
    'bg', [3, 1]
    'dm', [1, 1]
    'dc', [3, 1]
    'kF', [6, 1]
    'tauF', [1, 1]
    'cF', [6, 1]
    };
cursor = 1;
layout = struct();
layout.fields = fields(:, 1).';
for index = 1:size(fields, 1)
    name = fields{index, 1};
    shape = fields{index, 2};
    count = prod(shape);
    layout.(name) = struct('indices', cursor:(cursor + count - 1), 'shape', shape);
    cursor = cursor + count;
end
layout.totalCount = cursor - 1;
end

function vector = packVariables(variables, layout)
vector = zeros(layout.totalCount, 1);
for index = 1:numel(layout.fields)
    name = layout.fields{index};
    if ~isfield(variables, name)
        error('buildStewartFgMheVariables:MissingVariable', ...
            '缺少优化变量：%s。', name);
    end
    vector(layout.(name).indices) = variables.(name)(:);
end
end

function variables = unpackVariables(vector, layout)
vector = vector(:);
if numel(vector) ~= layout.totalCount
    error('buildStewartFgMheVariables:InvalidVectorLength', ...
        '优化向量长度应为 %d，实际为 %d。', layout.totalCount, numel(vector));
end
variables = struct();
for index = 1:numel(layout.fields)
    name = layout.fields{index};
    spec = layout.(name);
    variables.(name) = reshape(vector(spec.indices), spec.shape);
end
end
