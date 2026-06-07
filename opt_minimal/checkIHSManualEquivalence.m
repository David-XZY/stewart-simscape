function report = checkIHSManualEquivalence(standardData, manualData, zStd, zManual, outputFile)
% checkIHSManualEquivalence - 比较 standard IHSID 与 manual IHSID 在同一初值处的公平性
if nargin < 5
    outputFile = '';
end

[objectiveStd, gEqStd, cIneqStd] = evalNlp(standardData, zStd);
[objectiveManual, gEqManual, cIneqManual] = evalNlp(manualData, zManual);

report = struct();
report.objectiveStdAtX0 = objectiveStd;
report.objectiveManualAtX0 = objectiveManual;
report.absObjectiveDiffAtX0 = abs(objectiveStd - objectiveManual);
report.maxStdEqResidualAtX0 = maxAbs(gEqStd);
report.maxManualEqResidualAtX0 = maxAbs(gEqManual);
report.maxStdIneqViolationAtX0 = max([cIneqStd(:); 0]);
report.maxManualIneqViolationAtX0 = max([cIneqManual(:); 0]);
report.maxManualGapResidualAtX0 = checkFatropManualStructure(manualData, zManual).maxGapResidualAtX0;
report.maxManualMidResidualAtX0 = NaN;
report.maxManualHSResidualAtX0 = NaN;
report.maxManualDynResidualAtX0 = NaN;

if report.maxManualGapResidualAtX0 >= 1e-8
    error('checkIHSManualEquivalence:ManualGapFailed', ...
        'manual Yright 初值 gap 超过 1e-8。');
end
if report.absObjectiveDiffAtX0 > max(1e-8, 1e-8 * max(1, abs(objectiveStd)))
    error('checkIHSManualEquivalence:ObjectiveMismatch', ...
        'manual objective 与 standard objective 在同一初值处不一致。');
end

if ~isempty(outputFile)
    writeEquivalenceReport(outputFile, report);
end
end

function [objective, gEq, cIneq] = evalNlp(nlpData, z)
out = nlpData.eval('z', z);
objective = full(out.J);
gEq = full(out.gEq);
cIneq = full(out.cIneq);
end

function value = maxAbs(x)
if isempty(x)
    value = 0;
else
    value = max(abs(x(:)));
end
end

function writeEquivalenceReport(outputFile, report)
fid = fopen(outputFile, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'IHSID manual equivalence check\n');
names = fieldnames(report);
for i = 1:numel(names)
    value = report.(names{i});
    if isnumeric(value)
        fprintf(fid, '%s=%.16e\n', names{i}, value);
    else
        fprintf(fid, '%s=%s\n', names{i}, string(value));
    end
end
end
