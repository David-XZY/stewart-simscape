function reports=run_abfd_full_experiments(tasks,base,executionOverrides)
% Deferred full acceptance entry. Physical cases are never selected by outcome.
if nargin<3, executionOverrides=struct('caseId',{},'caseOverrides',{},'taskOverrides',{}); end
cases=buildAbfdCaseMatrix(); reports=cell(1,numel(cases));
assert(isstruct(executionOverrides) && all(isfield(executionOverrides,{'caseId','caseOverrides','taskOverrides'})) && ...
    numel(unique({executionOverrides.caseId}))==numel(executionOverrides) && ...
    all(ismember({executionOverrides.caseId},{cases.id})),'ABFD:FullMatrixOverrides','Unknown or duplicate physical case overrides.');
writeAbfdJson(fullfile(base.outputDir,'full_matrix_plan.json'), ...
    struct('cases',cases,'taskIds',{{tasks.taskId}},'executionOverrides',executionOverrides, ...
    'scope','All declared physical cases and tasks remain required regardless of completed outcomes.'));
for k=1:numel(cases)
    c=base; c.plantProfile=cases(k).profile; c.plantOverrides=cases(k).overrides;
    c.stateSource=cases(k).stateSource; c.outputDir=fullfile(base.outputDir,cases(k).id);
    selected=find(strcmp({executionOverrides.caseId},cases(k).id));
    if isempty(selected)
        reports{k}=run_abfd_matrix(tasks,c);
    else
        settings=executionOverrides(selected);
        reports{k}=run_abfd_matrix(tasks,c,settings.caseOverrides,settings.taskOverrides);
    end
end
generateAbfdReport(base.outputDir);
end
