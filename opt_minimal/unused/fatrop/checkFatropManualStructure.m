function report = checkFatropManualStructure(nlpData, z0, outputFile)
% checkFatropManualStructure - 求解前检查 IHSID manual FATROP 结构维度
if nargin < 3
    outputFile = '';
end

requiredFields = {'sizes', 'manualStructure', 'equalityMask', 'lbg', 'ubg'};
for i = 1:numel(requiredFields)
    if ~isfield(nlpData, requiredFields{i})
        error('checkFatropManualStructure:MissingField', ...
            'manual nlpData 缺少字段 %s。', requiredFields{i});
    end
end

N = nlpData.manualStructure.N;
nxVec = nlpData.manualStructure.nx * ones(N+1, 1);
nuVec = nlpData.manualStructure.nu(:);
ngVec = nlpData.manualStructure.ng(:);
numZExpected = sum(nxVec) + sum(nuVec);
numGExpected = numel(nlpData.lbg);
numEqExpected = sum(logical(nlpData.equalityMask(:)));
numIneqExpected = sum(~logical(nlpData.equalityMask(:)));

report = struct();
report.N = N;
report.nxVec = nxVec;
report.nuVec = nuVec;
report.ngVec = ngVec;
report.numZ_expected = numZExpected;
report.numZ_actual = nlpData.sizes.numZ;
report.numG_expected = numGExpected;
report.numG_actual = numel(nlpData.ubg);
report.numEq_expected = numEqExpected;
report.numEq_actual = nlpData.sizes.numEq;
report.numIneq_expected = numIneqExpected;
report.numIneq_actual = nlpData.sizes.numIneq;
report.equalityMaskLength = numel(nlpData.equalityMask);
report.equalityMaskTrue = numEqExpected;
report.equalityMaskFalse = numIneqExpected;
report.maxGapResidualAtX0 = computeGapResidual(z0, N, nlpData.manualStructure.nuTerminal);

assert(report.numZ_expected == report.numZ_actual, 'manual numZ 自检失败。');
assert(report.numG_expected == report.numG_actual, 'manual numG 自检失败。');
assert(report.numEq_expected == report.numEq_actual, 'manual 等式数量自检失败。');
assert(report.numIneq_expected == report.numIneq_actual, 'manual 不等式数量自检失败。');
assert(report.equalityMaskLength == report.numG_actual, 'manual equality mask 长度自检失败。');
assert(report.maxGapResidualAtX0 < 1e-8, 'manual Yright 初值 gap 自检失败。');

if ~isempty(outputFile)
    writeStructureReport(outputFile, report);
end
end

function maxResidual = computeGapResidual(z0, N, nuTerminal)
z0 = z0(:);
cursor = 0;
Y = zeros(24, N+1);
Yright = zeros(24, N);
for k = 1:N
    Y(:, k) = z0(cursor + (1:24));
    cursor = cursor + 24;
    uk = z0(cursor + (1:96));
    cursor = cursor + 96;
    Yright(:, k) = uk(73:96);
end
Y(:, N+1) = z0(cursor + (1:24));
cursor = cursor + 24 + nuTerminal; %#ok<NASGU>
gap = Y(:, 2:end) - Yright;
maxResidual = max(abs(gap(:)));
end

function writeStructureReport(outputFile, report)
fid = fopen(outputFile, 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'IHSID manual FATROP structure check\n');
fprintf(fid, 'N=%d\n', report.N);
fprintf(fid, 'numZ_expected=%d\nnumZ_actual=%d\n', report.numZ_expected, report.numZ_actual);
fprintf(fid, 'numG_expected=%d\nnumG_actual=%d\n', report.numG_expected, report.numG_actual);
fprintf(fid, 'numEq_expected=%d\nnumEq_actual=%d\n', report.numEq_expected, report.numEq_actual);
fprintf(fid, 'numIneq_expected=%d\nnumIneq_actual=%d\n', report.numIneq_expected, report.numIneq_actual);
fprintf(fid, 'length(equalityMask)=%d\n', report.equalityMaskLength);
fprintf(fid, 'sum(equalityMask)=%d\nsum(~equalityMask)=%d\n', ...
    report.equalityMaskTrue, report.equalityMaskFalse);
fprintf(fid, 'maxGapResidualAtX0=%.16e\n', report.maxGapResidualAtX0);
fprintf(fid, 'nuVec=');
fprintf(fid, ' %d', report.nuVec);
fprintf(fid, '\nngVec=');
fprintf(fid, ' %d', report.ngVec);
fprintf(fid, '\n');
end
