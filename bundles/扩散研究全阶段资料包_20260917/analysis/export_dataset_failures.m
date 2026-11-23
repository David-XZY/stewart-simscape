function rows=export_dataset_failures(runFolder)
% Explain rejected labels without changing their stored acceptance decisions.
files=dir(fullfile(runFolder,'attempts','*','start_*.mat'));
template=struct('taskId','','start',0,'reason','','rejectedStage','','violations','', ...
    'solverStatus','','maxEquality',NaN,'maxInequality',NaN,'minClearanceMm',NaN, ...
    'maxLegViolation',NaN,'maxForceViolation',NaN,'minSigma',NaN,'maxCondition',NaN);
rows=repmat(template,0,1);
for k=1:numel(files)
    s=load(fullfile(files(k).folder,files(k).name)); if s.record.passed, continue; end
    row=template; row.taskId=s.record.taskId; row.start=s.record.start; row.reason=s.record.reason; r=[];
    if ~isempty(s.result)
        row.solverStatus=s.result.solverStatus; row.maxEquality=s.result.maxEquality; row.maxInequality=s.result.maxInequality;
        row.rejectedStage='ihsid_or_posterior'; r=s.result.report;
        if s.result.passed
            row.rejectedStage='spline_fit'; r=s.fitReport;
        end
    else, row.rejectedStage='exception_before_result'; end
    if ~isempty(r)
        row.minClearanceMm=1000*value(r,'minClearance'); row.maxLegViolation=value(r,'maxLegViolation');
        row.maxForceViolation=value(r,'maxForceViolation'); row.minSigma=value(r,'minSigma'); row.maxCondition=value(r,'maxCondition');
        labels={};
        if row.minClearanceMm<10-1e-5, labels{end+1}='clearance'; end %#ok<AGROW>
        if row.maxLegViolation>1e-6, labels{end+1}='leg_kinematics'; end %#ok<AGROW>
        if row.maxForceViolation>1e-6, labels{end+1}='force'; end %#ok<AGROW>
        if row.minSigma<.1 || row.maxCondition>20, labels{end+1}='singularity'; end %#ok<AGROW>
        if value(r,'boundaryError')>1e-6, labels{end+1}='boundary'; end %#ok<AGROW>
        for name={'q','v','a','FRequired','FInterpolated'}
            if isfield(r,name{1}) && any(~isfinite(reshape(r.(name{1}),[],1))), labels{end+1}='nonfinite'; break; end %#ok<AGROW>
        end
        row.violations=strjoin(labels,';');
    end
    if isempty(row.violations), row.violations='see_solver_or_exception_reason'; end
    rows(end+1)=row; %#ok<AGROW>
end
writetable(struct2table(rows),fullfile(runFolder,'dataset_failures.csv'));
fprintf('DATASET_FAILURES_EXPORTED %d rejected of %d available attempt files\n',numel(rows),numel(files));
end
function x=value(s,name)
x=NaN; if isfield(s,name), x=s.(name); end
end
