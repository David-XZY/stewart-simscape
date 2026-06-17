function test_16_meeting_figure_export_contract
% test_16_meeting_figure_export_contract - 验证组会结果图的 FIG/PNG 导出契约
testRoot = fileparts(mfilename('fullpath'));
controllerRoot = fileparts(testRoot);
projectRoot = fileparts(controllerRoot);
optRoot = fullfile(projectRoot, 'opt_minimal');
addpath(fullfile(controllerRoot, 'tools'));
outputDir = fullfile(tempdir, 'stewart_run02_run03_meeting_figures');
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
cleanup = onCleanup(@() removeDirectory(outputDir));

result = exportRun02Run03MeetingFigures("", "", outputDir);
assert(numel(result.figFiles) == 4);
assert(numel(result.pngFiles) == 4);
assert(all(isfile(result.figFiles)));
assert(all(isfile(result.pngFiles)));
assert(isfield(result, 'rippleDiagnostics'));
diagnostics = result.rippleDiagnostics;
assert(isscalar(diagnostics.legIndex) && diagnostics.legIndex >= 1 && diagnostics.legIndex <= 6);
assert(isequal(size(diagnostics.zoomWindow), [1, 2]));
assert(all(isfinite(diagnostics.zoomWindow)));
assert(isfinite(diagnostics.dominantFrequencyHz) && diagnostics.dominantFrequencyHz > 0);
assert(isfinite(diagnostics.referenceNodeFrequencyHz) && diagnostics.referenceNodeFrequencyHz > 0);
assert(isfinite(diagnostics.nodeFrequencyAmplitudeNormalized));
assert(isfinite(diagnostics.innerBandwidthHz) && diagnostics.innerBandwidthHz > 0);
assert(abs(diagnostics.sampleFrequencyHz - 100) < 1e-9);

for index = 1:4
    fig = openfig(result.figFiles{index}, 'invisible');
    figCleanup = onCleanup(@() close(fig));
    assert(~isempty(findall(fig, 'Type', 'axes')));
    clear figCleanup;

    imageInfo = imfinfo(result.pngFiles{index});
    aspectRatio = imageInfo.Width / imageInfo.Height;
    assert(abs(aspectRatio - 16/9) < 0.03);
    assert(imageInfo.Width >= 4000);
end
end

function removeDirectory(outputDir)
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
end
