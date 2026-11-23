function report=pgExpansionRawLibrary()
% Safe raw trajectories remain accepted even if 16-control encoding fails.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here)); addpath(fullfile(root,'chapter3','payload_generalization'));
c=pgConfig('payload_condition_expansion_v2_longtime_20260926'); data=jsondecode(readUtf8(fullfile(c.output,'dataset.json')));
library=jsondecode(readUtf8(fullfile(c.output,'library.json'))); known={data.rows.taskId};
records=num2cell(library.records); rawRows={}; checks=dir(fullfile(c.output,'checks','*.json'));
for i=1:numel(checks)
    r=jsondecode(readUtf8(fullfile(checks(i).folder,checks(i).name)));
    if r.accepted || ~isfield(r,'rawPhysical') || ~r.rawPhysical.passed || ismember(r.taskId,known), continue; end
    s=load(r.source); task=pgFinalizeTask(s.task,s.task.payload,c); a=pgValidate(s.sol.bundle,task,c);
    assert(a.passed,'Expansion:RawAudit','Raw trajectory failed repeat physical audit');
    id=sprintf('raw_accepted_%05d',numel(rawRows)+1);
    record=struct('labelId',id,'task',task,'bundle',s.sol.bundle,'fitted',[],'fittedPassed',false,'source',r.source, ...
        'objective',s.sol.objective,'sourceSolverPassed',s.sol.passed,'sourceSolverStatus',s.sol.solverStatus);
    save(fullfile(c.output,[id,'.mat']),'record','-v7');
    records{end+1}=struct('labelId',id,'taskId',task.taskId,'split','train','file',[id,'.mat'], ...
        'rawPassed',true,'fittedPassed',false,'source',r.source);
    rawRows{end+1}=struct('taskId',task.taskId,'origin',r.origin,'source',r.source,'sourceSha256',bdFileHash(r.source), ...
        'file',[id,'.mat'],'physicalPassed',true,'physicalReason',a.reason,'encodingReason',r.reason, ...
        'minClearance',a.minClearance,'minSigma',a.minSigma,'maxCondition',a.maxCondition, ...
        'boundaryError',a.boundaryError,'sampleCount',a.sampleCount,'maxDenseDynamicResidual',a.maxDenseDynamicResidual);
    known{end+1}=task.taskId;
end
trainLabels=sum(strcmp({data.rows.split},'train'));
report=struct('status','completed','trainingSplineConditions',trainLabels,'additionalRawSafeConditions',numel(rawRows), ...
    'physicallyAcceptedTrainConditions',trainLabels+numel(rawRows),'oneTrajectoryPerCondition',true,'rawOnly',{rawRows});
dpWriteJson(fullfile(c.output,'physical_trajectory_library.json'),struct('version','physical_trajectory_library_v1', ...
    'records',{records},'report',report,'note','Every physical trajectory accepted; raw-only entries are not encoded as 16-control training labels.'));
dpWriteJson(fullfile(c.output,'physical_acceptance_report.json'),report);
end
function s=readUtf8(file)
fid=fopen(file,'r','n','UTF-8'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); s=fread(fid,inf,'*char').';
end
