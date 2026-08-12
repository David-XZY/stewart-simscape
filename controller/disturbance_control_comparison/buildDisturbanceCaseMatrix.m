function cases = buildDisturbanceCaseMatrix(config, selection)
% buildDisturbanceCaseMatrix - Expand the canonical 40-case experiment matrix.
arguments
    config struct
    selection {mustBeMember(selection, {'all', 'training', 'holdout'})} = 'all'
end

template = struct('id', "", 'disturbanceType', "", 'scale', 0, ...
    'noiseSeed', NaN, 'hasWrench', false, 'hasSmoothBump', false, ...
    'hasTargetNoise', false, 'isTraining', false, 'isHoldout', false);
items = template;
items.id = "nominal";
items.disturbanceType = "nominal";
items.isTraining = true;

for scale = config.disturbanceScales(:).'
    items(end+1) = makeCase(template, "wrench", scale, NaN, true, false, false, config); %#ok<AGROW>
    items(end+1) = makeCase(template, "smooth_target", scale, NaN, false, true, false, config); %#ok<AGROW>
    items(end+1) = makeCase(template, "wrench_smooth_target", scale, NaN, true, true, false, config); %#ok<AGROW>
    for seed = config.noiseSeeds(:).'
        items(end+1) = makeCase(template, "target_noise", scale, seed, false, false, true, config); %#ok<AGROW>
        items(end+1) = makeCase(template, "wrench_target_noise", scale, seed, true, false, true, config); %#ok<AGROW>
    end
end

switch selection
    case 'training'
        cases = items([items.isTraining]);
    case 'holdout'
        cases = items([items.isHoldout]);
    otherwise
        cases = items;
end
cases = cases(:);
end

function value = makeCase(template, type, scale, seed, wrench, bump, noise, config)
value = template;
value.disturbanceType = string(type);
value.scale = scale;
value.noiseSeed = seed;
value.hasWrench = wrench;
value.hasSmoothBump = bump;
value.hasTargetNoise = noise;
scaleCode = replace(compose('%.1f', scale), '.', 'p');
if noise
    value.id = string(type)+"_x"+scaleCode+"_s"+string(seed);
else
    value.id = string(type)+"_x"+scaleCode;
end
isMedium = abs(scale-config.trainingScale) < 1e-12;
if noise
    value.isTraining = isMedium && ismember(seed, config.trainingNoiseSeeds);
    value.isHoldout = ~value.isTraining;
else
    value.isTraining = isMedium;
    value.isHoldout = ~isMedium;
end
end
