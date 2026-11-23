try
    projectRoot = 'D:\NPU\Matlab_code\stewart-simscape-disturbance-experiment';
    cd(projectRoot);
    addpath(fullfile(projectRoot, 'controller', 'runs'));
    options = struct('useFastRestart', true, 'failOnRejected', false, ...
        'integrationSubsteps', 1);
    result = run_15_disturbance_control_comparison('', '', options); %#ok<NASGU>
    fprintf('\n===== FINAL EXACT RANKING =====\n');
    disp(result.exactRanking);
    fprintf('\n===== FINAL SIMSCAPE RANKING =====\n');
    disp(result.simscape.ranking);
    fprintf('\nFULL_EXPERIMENT_COMPLETED\n');
catch exception
    fprintf(2, '\nFULL_EXPERIMENT_FAILED\n%s\n', ...
        getReport(exception, 'extended', 'hyperlinks', 'off'));
    rethrow(exception);
end
