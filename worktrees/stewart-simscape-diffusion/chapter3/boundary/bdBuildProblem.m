function info=bdBuildProblem(c)
% Resident warmup on a fixed, task-independent prototype; no trajectory solve.
task=bdMakeTask([],[0,0,0,7.5],c); c.buildOnly=true; c.clockHandle=tic;
info=bdSolve(task,bdBaseline(task,c.controlCount),c);
end
