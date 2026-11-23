function summary=bdReferenceSmoke()
c=bdConfig('boundary_reference_smoke_v1'); root=c.root;
source=fullfile(root,'results','chapter3','boundary_geometric_probe_v1','feasible_050.mat');
z=load(source); task=z.task; cp=z.bundle.cp;
warmClock=tic; bdBuildProblem(c); warmSeconds=toc(warmClock);
c.clockHandle=tic; c.wallBudget=120; c.maxCpuSeconds=120; c.debugInitial=true;
result=bdSolve(task,cp,c); record=struct('passed',result.passed,'reason',result.solverStatus,'warmupSeconds',warmSeconds, ...
    'buildSeconds',result.buildSeconds,'solveSeconds',result.solveSeconds,'validationSeconds',result.validationSeconds, ...
    'maxEquality',result.maxEquality,'maxInequality',result.maxInequality,'maxBounds',result.maxBounds);
if result.passed
    record.quality=bdQuality(result.bundle,task,c); bdExportTrajectory(result.bundle,task,fullfile(c.output,'plot_data.json'),c);
end
save(fullfile(c.output,'reference.mat'),'task','result','record','-v7'); dpWriteJson(fullfile(c.output,'reference.json'),record);
% Repair interface is tested on the independent, already feasible seed.
c.returnPolicy='quality'; c.qualityFrozen=true; c.positionCap=.1; c.rotationCap=.1;
clock=tic; [repaired,repair]=bdRepair(z.bundle,task,clock,3,c);
record.repairSmoke=repair;
summary=record; dpWriteJson(fullfile(c.output,'summary.json'),summary);
fprintf('BD_REFERENCE_SMOKE passed=%d repairIterations=%d\n',result.passed,repair.iterations);
end
