function test_16_meeting_figure_export_contract
% test_16_meeting_figure_export_contract - 验证组会结果图的 FIG/PNG 导出契约
optRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(optRoot, 'tools'));
outputDir = fullfile(tempdir, 'stewart_run02_run03_meeting_figures');
if isfolder(outputDir)
    rmdir(outputDir, 's');
end
cleanup = onCleanup(@() removeDirectory(outputDir));

result = exportRun02Run03MeetingFigures("", "", outputDir);
assert(numel(result.figFiles) == 3);
assert(numel(result.pngFiles) == 3);
assert(all(isfile(result.figFiles)));
assert(all(isfile(result.pngFiles)));

for index = 1:3
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
