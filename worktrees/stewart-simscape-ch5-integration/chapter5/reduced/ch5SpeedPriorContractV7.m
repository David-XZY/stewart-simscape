function report=ch5SpeedPriorContractV7()
clear ch5ReducedControlV7 ch5ReducedAuditFastV7 ch5ReducedAuditV7 ch5ReducedV7Sfun
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','prior_contract_v7');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'identity',ch5ReducedIdentity(),'pid',feature('getpid'));
for mode=["ideal-force","nonideal-force"]
    r=ch5ReducedPrepare(1,mode);r.reduced=struct('iterations',1,'backtracking',5,'factor_derivatives',false);
    if isappdata(0,'ch5ReducedExecution'),rmappdata(0,'ch5ReducedExecution');end
    setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',5101));
    [goal,o]=ch5Observe(r.task,0);x=[r.task.q0;r.task.qd0];if r.plant.n>12,x=[x;repmat(r.initialForce,3,1)];end
    [u0,d0,p0]=ch5ReducedControlV7(x,r.initialForce,goal,o,r,0);
    x1=full(r.plant.fd(x,u0,zeros(6,1)));[goal,o]=ch5Observe(r.task,r.c.Ts);
    [u1,d1,p1]=ch5ReducedControlV7(x1,u0,goal,o,r,r.c.Ts);
    k=ch5ReducedKernelsV4(r);U=p0.candidates{1}.U;prior=ones(size(U))*16;
    [~,old]=ch5ReducedStepV6(x,r.initialForce,U,goal,o,prior,r,k,true);
    [~,now]=ch5ReducedStepV7(x,r.initialForce,U,goal,o,prior,r,k,true);
    nv=numel(old.f);nr=numel(old.b);
    matrixErrors=[norm(now.Q(1:nv,1:nv)-old.Q,inf)/(1+norm(old.Q,inf)),norm(now.f(1:nv)-old.f,inf)/(1+norm(old.f,inf)), ...
        norm(now.A(1:nr,1:nv)-old.A,inf),norm(now.b(1:nr)-old.b,inf),norm(now.E(:,1:nv)-old.E,inf),norm(now.e-old.e,inf)];
    correction=sin((1:nv)')*.01;z=abs(now.offset+now.offset_matrix*correction);augmented=[correction;z];
    epigraphViolation=max([now.A(nr+1:end,:)*augmented-now.b(nr+1:end);0]);
    extraCost=.5*augmented'*now.Q*augmented+now.f'*augmented-(.5*correction'*old.Q*correction+old.f'*correction);
    epigraphCostError=abs(extraCost-100*sum(z)-.5e-6*sum(z.^2));
    [~,trajectory,cost0]=ch5ReducedAuditV7(x,r.initialForce,U,goal,o,r,k,zeros(size(U)));
    [~,~,costP]=ch5ReducedAuditV7(x,r.initialForce,U,goal,o,r,k,prior);
    expectedCost=.05*r.c.Ts*sum(((trajectory.delta-prior)/160).^2-(trajectory.delta/160).^2,'all');
    costError=abs(costP-cost0-expectedCost);
    solverSuccess=p0.denoise_attempts{1}.solver_success;
    row=struct('mode',char(mode),'first_refresh',p0.refresh,'next_refresh',p1.refresh,'first_feasible',d0.feasible, ...
        'next_feasible',d1.feasible,'first_cost',p0.cost,'next_cost',p1.cost, ...
        'matrix_errors',matrixErrors,'prior_cost_error',costError,'epigraph_violation',epigraphViolation,'epigraph_cost_error',epigraphCostError,'predictive_solver_success',solverSuccess, ...
        'accepted',p0.refresh&&~p1.refresh&&d0.feasible&&d1.feasible&&solverSuccess&&all(isfinite([u0;u1]))&&isfinite(p0.cost)&&isfinite(p1.cost)&&max(matrixErrors)<1e-8&&costError<1e-9&&epigraphViolation<1e-9&&epigraphCostError<1e-8);
    report.rows{end+1}=row;fprintf('Two-step prior contract %s accepted=%d\n',mode,row.accepted);
end
report.accepted=all(cellfun(@(s)s.accepted,report.rows));
save(fullfile(directory,'audit.mat'),'report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.accepted,'ch5:PriorContract','Consecutive refresh and intermediate execution calls must remain valid.');
end
