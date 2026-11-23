function report=ch5SpeedNumericsAuditV33()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'numerics_audit_v33');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'accepted',true,'error','');
try
    tail=jsondecode(fileread(fullfile(base,'tail_audit_v23','summary.json')));numeric=jsondecode(fileread(fullfile(base,'restructure_audit_v24','summary.json')));
    report.tail_accepted=tail.accepted;report.solver_cases_accepted=all(cellfun(@(v)v.accepted,numeric.solver_rows));
    old=fileread(fullfile(root,'chapter5','reduced','ch5ReducedControlV22.m'));
    expected=strrep(strrep(strrep(strrep(old,'ch5ReducedControlV22','ch5ReducedControlV33'),'ch5ReducedWarmV11','ch5ReducedWarmV23'),'ch5ReducedAuditV22','ch5ReducedAuditV23'),'ch5ReducedStepV20','ch5ReducedStepV33');
    report.controller_scope_exact=strcmp(expected,fileread(fullfile(root,'chapter5','reduced','ch5ReducedControlV33.m')));
    old=fileread(fullfile(root,'chapter5','reduced','ch5ReducedStepV20.m'));
    expected=strrep(strrep(old,'ch5ReducedStepV20','ch5ReducedStepV33'),'ch5ReducedSolveV19(linear,"whiten-active")','ch5ReducedSolveV24(linear,r)');
    report.step_scope_exact=strcmp(expected,fileread(fullfile(root,'chapter5','reduced','ch5ReducedStepV33.m')));
    files={'ch5ReducedControlV33.m','ch5ReducedStepV33.m','ch5ReducedV33Sfun.m','ch5ReducedSolveV24.m','ch5ReducedTerminalV23.m','ch5ReducedWarmV23.m','ch5ReducedAuditV23.m'};
    report.syntax_rows={};
    for j=1:numel(files)
        issues=checkcode(fullfile(root,'chapter5','reduced',files{j}),'-id');ids={issues.id};
        syntax=any(strcmp(ids,'SYNER'));report.syntax_rows{end+1}=struct('file',files{j},'syntax_error',syntax,'issues',issues);
        assert(~syntax,'ch5:SyntaxFailure','New numerical backend failed syntax validation.');
    end
    report.accepted=report.tail_accepted&&report.solver_cases_accepted&&report.controller_scope_exact&&report.step_scope_exact;
catch ex,report.accepted=false;report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);assert(report.accepted,'ch5:NumericsUnverified','Numerical-only scope must be audited before Simscape.');
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
