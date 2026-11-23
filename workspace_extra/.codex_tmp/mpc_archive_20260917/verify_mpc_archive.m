workRoot = fileparts(mfilename('fullpath'));
projectRoot = fullfile(workRoot, 'MPC_代码与结果_20260917');
restoredefaultpath;
cd(projectRoot);
addpath(genpath(projectRoot));
verification = struct('passed', false, 'matlabVersion', version, 'tests', [], ...
    'originalArtifactsReadable', false, 'physicalClosedLoopValidated', false);
try
    fprintf('Verification scope: archived prototype, original files unchanged.\n');
    fprintf('This does not validate a physical closed loop or superiority over LQI/QP.\n');
    targetTemp = fullfile(tempdir, 'sampling_mpc_export_contract');
    expectedTemp = fullfile(workRoot, 'matlab_temp');
    assert(startsWith(lower(char(java.io.File(targetTemp).getCanonicalPath())), ...
        lower([char(java.io.File(expectedTemp).getCanonicalPath()), filesep])));
    assert(~isfolder(targetTemp), 'Test temporary output must not already exist.');
    tests = {'test_50_sampling_mpc_config_contract', ...
        'test_51_sampling_mpc_step_contract', 'test_52_sampling_mpc_comparison_export'};
    testResults = repmat(struct('name', '', 'passed', false), 1, numel(tests));
    for k = 1:numel(tests)
        feval(tests{k});
        testResults(k) = struct('name', tests{k}, 'passed', true);
        fprintf('PASS %s\n', tests{k});
    end
    verification.tests = testResults;
    artifactRoot = fullfile(projectRoot, 'results', 'reports', 'sampling_mpc_control');
    saved = load(fullfile(artifactRoot, 'sampling_mpc_comparison_data.mat'));
    assert(isfield(saved, 'comparison') && isfield(saved, 'config'));
    assert(numel(saved.comparison.runs) == 4);
    values = readtable(fullfile(artifactRoot, 'table_sampling_mpc_comparison.csv'));
    assert(height(values) == 4);
    for k = 1:4
        runData = saved.comparison.runs(k);
        assert(isequal(size(runData.pose), [6, numel(runData.time)]));
        assert(all(isfinite(runData.pose), 'all'));
        calculated = sqrt(mean(sum(runData.poseError(1:3, :).^2, 1)));
        assert(abs(calculated - values.position_rmse(k)) < 1e-12);
    end
    pngs = dir(fullfile(artifactRoot, '*.png'));
    assert(numel(pngs) == 6);
    for k = 1:numel(pngs)
        pixels = imread(fullfile(pngs(k).folder, pngs(k).name));
        assert(~isempty(pixels));
    end
    figs = dir(fullfile(artifactRoot, '*.fig'));
    assert(numel(figs) == 6);
    for k = 1:numel(figs)
        fig = openfig(fullfile(figs(k).folder, figs(k).name), 'invisible');
        assert(isgraphics(fig));
        close(fig);
    end
    verification.originalArtifactsReadable = true;
    verification.savedGeneratedAt = char(saved.comparison.generatedAt);
    verification.savedSampleTime = saved.config.sampleTime;
    verification.savedHorizon = saved.config.horizonSteps;
    verification.savedCandidateCount = numel(saved.config.candidateScales);
    verification.savedSamples = numel(saved.comparison.runs(4).time);
    verification.savedMpcMeanSolveSeconds = values.mean_solve_time(4);
    verification.savedMpcMaxSolveSeconds = values.max_solve_time(4);
    entry = fullfile(projectRoot, 'controller', 'runs', 'run_10_sampling_mpc_control.m');
    [dependencies, products] = matlab.codetools.requiredFilesAndProducts(entry);
    verification.dependencyCount = numel(dependencies);
    verification.requiredProducts = {products.Name};
    for k = 1:numel(dependencies)
        assert(startsWith(lower(dependencies{k}), lower(projectRoot)), ...
            'A project dependency is outside the archive: %s', dependencies{k});
    end
    verification.passed = true;
    fprintf('PASS original MAT/CSV metrics agree; 6 PNG and 6 FIG readable.\n');
    fprintf('PASS entrypoint dependency scan: %d files.\n', numel(dependencies));
catch err
    verification.error = getReport(err, 'extended', 'hyperlinks', 'off');
    fid = fopen(fullfile(workRoot, 'verification.json'), 'w', 'n', 'UTF-8');
    fprintf(fid, '%s', jsonencode(verification, PrettyPrint=true));
    fclose(fid);
    rethrow(err);
end
fid = fopen(fullfile(workRoot, 'verification.json'), 'w', 'n', 'UTF-8');
fprintf(fid, '%s', jsonencode(verification, PrettyPrint=true));
fclose(fid);
