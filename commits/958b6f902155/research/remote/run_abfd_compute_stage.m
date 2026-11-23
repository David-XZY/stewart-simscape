function run_abfd_compute_stage(stage, outputDir, solverMaxIterations)
% Explicit compute stages. Pilot outputs are partial data, never trained assets.
root = setupAbfdResearch();
if nargin < 2, outputDir = fullfile(root,'results','abfd','compute'); end
if nargin < 3, solverMaxIterations=300; end
if ~isfolder(outputDir), mkdir(outputDir); end
statusFile = fullfile(outputDir,[stage,'_status.json']);
status = struct('stage',stage,'state','running','started',timestamp(), ...
    'matlab',version,'root',root,'solverMaxIterations',solverMaxIterations,'message','');
writeAbfdJson(statusFile,status);
try
    switch stage
        case {'F-pilot','F-data'}
            tasks = makeAbfdTaskManifest('proposal',60,20260905);
            c = makeAbfdConfig(struct('outputDir',fullfile(outputDir,'F1'), ...
                'solverMaxIterations',solverMaxIterations,'solverMaxCpuTime',600, ...
                'plannerProfile','guarded-exact-v1'));
            writeAbfdJson(fullfile(outputDir,'F_tasks_frozen.json'),tasks);
            writeAbfdJson(fullfile(outputDir,'F_config.json'),c);
            if strcmp(stage,'F-pilot'), tasks=tasks(1:3); end
            run_F1_generate_multistart_dataset(tasks,c);
            if strcmp(stage,'F-data')
                run_F0_multimodality_audit(fullfile(c.outputDir,'proposal_dataset.json'), ...
                    fullfile(c.outputDir,'multimodality_audit.json'));
            end
        case 'B-pilot'
            tasks = makeAbfdTaskManifest('residual',60,20260905);
            writeAbfdJson(fullfile(outputDir,'B_tasks_frozen.json'),tasks);
            task=tasks(1);
            c=makeAbfdConfig(struct('outputDir',fullfile(outputDir,'B_pilot'), ...
                'solverMaxIterations',solverMaxIterations,'solverMaxCpuTime',600, ...
                'plantProfile','high-fidelity','plantOverrides',task.plantOverrides));
            result=runAbfdExperiment(task,c);
            assert(~isempty(result.log),'ABFD:EmptyPilotLog','No actual plant log.');
            quality=validateResidualCollectionRun(result,task.model,c.constraintTolerance);
            status.engineeringPassed=quality.engineeringPassed; status.budgetPassed=quality.budgetPassed;
        case 'B-data'
            tasks=makeAbfdTaskManifest('residual',60,20260905);
            writeAbfdJson(fullfile(outputDir,'B_tasks_frozen.json'),tasks);
            c=makeAbfdConfig(struct('outputDir',fullfile(outputDir,'B0'), ...
                'solverMaxIterations',solverMaxIterations,'solverMaxCpuTime',600));
            run_B0_collect_dataset(tasks,c);
        otherwise
            error('ABFD:UnknownStage','Unknown compute stage: %s',stage);
    end
    status.state='complete';
catch exception
    status.state='failed'; status.message=getReport(exception,'extended','hyperlinks','off');
    status.finished=timestamp(); writeAbfdJson(statusFile,status); rethrow(exception);
end
status.finished=timestamp(); writeAbfdJson(statusFile,status);
end
function value=timestamp()
value=char(datetime('now','Format','yyyyMMdd''T''HHmmss'));
end
