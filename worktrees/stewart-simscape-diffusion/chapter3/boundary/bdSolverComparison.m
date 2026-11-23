function summary=bdSolverComparison()
% Debug-only matched legacy/new solver calls, same feasible seed and objective.
% Does not join formal method timings or label generation.
c=bdConfig('boundary_solver_check_v2'); z=load(fullfile(c.root,'results','chapter3','boundary_geometric_probe_v1','feasible_050.mat'));
task=z.task; records={};
for variant={'legacy','parameterized'}
    c.clockHandle=tic; c.wallBudget=90; c.maxCpuSeconds=60;
    if strcmp(variant{1},'legacy'), result=c3Solve(task,z.bundle.cp,c);
    else, result=bdSolve(task,z.bundle.cp,c); end
    independent=bdValidate(result.bundle,task,c,tic,30);
    record=struct('variant',variant{1},'passed',result.passed&&independent.passed,'solverStatus',result.solverStatus, ...
        'iterations',result.iterations,'buildSeconds',result.buildSeconds,'solveSeconds',result.solveSeconds, ...
        'maxEquality',result.maxEquality,'maxInequality',result.maxInequality,'objective',result.objective, ...
        'physicalReason',independent.reason,'codeHash',c.codeHash);
    records{end+1}=record; save(fullfile(c.output,[variant{1},'.mat']),'task','result','record','-v7');
    dpWriteJson(fullfile(c.output,[variant{1},'.json']),record);
end
clock=tic; [bundle,repair]=bdRepair(z.bundle,task,clock,3,c);
summary=struct('records',{records},'repairSmoke',repair,'scope','Debug comparisons only; full acceptance remains mandatory.');
dpWriteJson(fullfile(c.output,'summary.json'),summary);
end
