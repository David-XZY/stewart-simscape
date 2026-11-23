function rows=export_acceptance(runFolder)
% Portable tabular view of real MAT reports, including failures.
files=dir(fullfile(runFolder,'benchmark','*','*','benchmark.mat')); rows=[];
for k=1:numel(files)
    folder=files(k).folder; s=load(fullfile(folder,files(k).name)); result=[];
    origin='no_solver_result'; candidate=NaN;
    if ~isempty(s.outcome) && ~isempty(s.outcome.attempts)
        candidate=s.outcome.attempts(end).candidate;
        if s.row.passed
            result=s.outcome.best; origin='accepted_result';
        else
            last=load(fullfile(folder,sprintf('candidate_%02d.mat',candidate)),'result');
            result=last.result; origin='last_failed_attempt';
        end
    end
    row=struct('taskId',s.task.taskId,'family',s.task.family,'method',s.row.method, ...
        'passed',s.row.passed,'onlineSeconds',s.row.seconds,'solveAttempts',s.row.solveAttempts, ...
        'diagnosticSource',origin,'candidate',candidate,'solverStatus','', ...
        'maxEquality',NaN,'maxInequality',NaN,'minClearanceMm',NaN,'maxLegViolation',NaN, ...
        'maxForceViolation',NaN,'minSigma',NaN,'maxCondition',NaN,'boundaryError',NaN, ...
        'maxDenseDynamicResidual',NaN,'denseSamples',NaN,'reason',s.row.reason);
    if ~isempty(result)
        r=result.report; row.solverStatus=result.solverStatus;
        row.maxEquality=result.maxEquality; row.maxInequality=result.maxInequality;
        row.minClearanceMm=1000*r.minClearance; row.maxLegViolation=r.maxLegViolation;
        row.maxForceViolation=r.maxForceViolation; row.minSigma=r.minSigma; row.maxCondition=r.maxCondition;
        row.boundaryError=r.boundaryError; row.maxDenseDynamicResidual=r.maxDenseDynamicResidual; row.denseSamples=r.sampleCount;
        if s.row.passed
            assert(result.passed && r.passed && result.solverSuccess);
            assert(result.maxEquality<=1e-6 && result.maxInequality<=1e-6 && r.minClearance>=.01-1e-8);
            assert(r.maxLegViolation<=1e-6 && r.maxForceViolation<=1e-6 && r.boundaryError<=1e-6);
            assert(r.minSigma>=.1 && r.maxCondition<=20);
        end
    else
        assert(~s.row.passed,'Accepted benchmark is missing its solver result.');
    end
    if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
end
assert(~isempty(rows),'No benchmark records to export.');
writetable(struct2table(rows),fullfile(runFolder,'acceptance.csv'));
fprintf('ACCEPTANCE_EXPORTED %d records, %d passed\n',numel(rows),sum([rows.passed]));
end
