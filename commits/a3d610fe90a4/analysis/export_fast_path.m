function export_fast_path(folder)
% Independent scalar readback of saved raw, repaired and accepted reports.
files=dir(fullfile(folder,'*','measurement.mat')); assert(numel(files)==20);
reference=jsondecode(fileread(fullfile(fileparts(folder),'formal_v1','benchmark_complete.json')));
candidates=[]; accepted=[]; repairs=[];
for i=1:numel(files)
    s=load(fullfile(files(i).folder,files(i).name));
    assert(strcmp(s.protocol.implementationHash,reference.implementationHash),'Numerical implementation differs.');
    assert(strcmp(s.protocol.modelHash,reference.assetHash),'Model differs.');
    assert(numel(s.direct)==8);
    for k=1:8
        e=s.direct(k); row=flatten(s.task,e.report,e.passed,e.candidate,e.seconds,'original');
        if isempty(candidates), candidates=row; else, candidates(end+1)=row; end %#ok<AGROW>
    end
    for k=1:numel(s.repairs)
        e=s.repairs(k); row=flatten(s.task,e.report,e.passed,e.candidate,e.seconds,'bounded_repair');
        if isempty(repairs), repairs=row; else, repairs(end+1)=row; end %#ok<AGROW>
    end
    if s.row.finalPassed
        row=flatten(s.task,s.finalReport,true,s.row.chosenCandidate,s.row.onlineSeconds,s.row.acceptedStage);
        if isempty(accepted), accepted=row; else, accepted(end+1)=row; end %#ok<AGROW>
    end
end
writetable(struct2table(candidates,'AsArray',true),fullfile(folder,'candidate_acceptance.csv'));
writetable(struct2table(accepted,'AsArray',true),fullfile(folder,'final_acceptance.csv'));
if ~isempty(repairs), writetable(struct2table(repairs,'AsArray',true),fullfile(folder,'repair_acceptance.csv')); end
fprintf('FASTPATH_REPORTS raw=%d passing=%d repairAttempts=%d finalAccepted=%d\n', ...
    numel(candidates),sum([candidates.passed]),numel(repairs),numel(accepted));
end
function row=flatten(task,r,passed,k,seconds,stage)
row=struct('taskId',task.taskId,'family',task.family,'stage',stage,'candidate',k,'passed',passed, ...
    'seconds',seconds,'minClearanceMm',NaN,'maxLegViolation',NaN,'maxForceViolation',NaN, ...
    'minSigma',NaN,'maxCondition',NaN,'boundaryError',NaN,'sampleCount',NaN);
if isempty(r), assert(~passed); return; end
for name={'maxLegViolation','maxForceViolation','minSigma','maxCondition','boundaryError','sampleCount'}
    if isfield(r,name{1}), row.(name{1})=r.(name{1}); end
end
if isfield(r,'minClearance'), row.minClearanceMm=1000*r.minClearance; end
if passed
    assert(r.passed && r.minClearance>=.01-1e-8 && r.maxLegViolation<=1e-6 && ...
        r.maxForceViolation<=1e-6 && r.minSigma>=.1 && r.maxCondition<=20 && r.boundaryError<=1e-6);
    assert(all(isfinite([r.q(:);r.v(:);r.a(:);r.FRequired(:);r.FInterpolated(:)])));
end
end
