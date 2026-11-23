function r=pgSolverSmoke(runName)
if nargin<1, runName='payload_solver_smoke_v2'; end
c=pgConfig(runName); base=bdMakeTask([],[0,0,0,7.5],c);
base.scene.hood.obstacles=base.scene.hood.obstacles([]); base.family='open';
t=pgFinalizeTask(base,pgMakePayload('cylinder',[1.2,.15],100),c);
c.clockHandle=tic; c.wallBudget=60; c.maxCpuSeconds=60;
r=pgSolve(t,bdBaseline(t),c); save(fullfile(c.output,'smoke.mat'),'r','t','-v7');
summary=rmfield(r,{'bundle','warm','sourceSeed','report'}); summary.physical=r.report.passed; summary.physicalReason=r.report.reason;
dpWriteJson(fullfile(c.output,'smoke.json'),summary);
end
