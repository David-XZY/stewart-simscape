function summary = summarizeStrictActiveConstraints(runs)
% summarizeStrictActiveConstraints - Count active constraints by scenario.
scenarioId = strings(0, 1);
constraintName = strings(0, 1);
activeSampleCount = zeros(0, 1);
activeSampleFraction = zeros(0, 1);
for runIndex = 1:numel(runs)
    sampleNames = runs(runIndex).diagnostics.activeConstraints;
    allNames = strings(0, 1);
    for sampleIndex = 1:numel(sampleNames)
        names = unique(string(sampleNames{sampleIndex}(:)));
        allNames = [allNames; names]; %#ok<AGROW>
    end
    if isempty(allNames)
        scenarioId(end+1, 1) = runs(runIndex).scenario.id; %#ok<AGROW>
        constraintName(end+1, 1) = "none"; %#ok<AGROW>
        activeSampleCount(end+1, 1) = 0; %#ok<AGROW>
        activeSampleFraction(end+1, 1) = 0; %#ok<AGROW>
        continue;
    end
    uniqueNames = unique(allNames);
    for nameIndex = 1:numel(uniqueNames)
        count = sum(allNames == uniqueNames(nameIndex));
        scenarioId(end+1, 1) = runs(runIndex).scenario.id; %#ok<AGROW>
        constraintName(end+1, 1) = uniqueNames(nameIndex); %#ok<AGROW>
        activeSampleCount(end+1, 1) = count; %#ok<AGROW>
        activeSampleFraction(end+1, 1) = count/numel(sampleNames); %#ok<AGROW>
    end
end
summary = table(scenarioId, constraintName, activeSampleCount, ...
    activeSampleFraction);
end
