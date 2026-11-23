function report=ch5SpeedPriorContract()
clear ch5ReducedControlV4 ch5ReducedAuditFastV4 ch5ReducedAuditV4 ch5ReducedV4Sfun
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','prior_contract_v5');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'identity',ch5ReducedIdentity(),'pid',feature('getpid'));
for mode=["ideal-force","nonideal-force"]
    r=ch5ReducedPrepare(1,mode);r.reduced=struct('iterations',1,'backtracking',5,'factor_derivatives',false);
    if isappdata(0,'ch5ReducedExecution'),rmappdata(0,'ch5ReducedExecution');end
    setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',5101));
    [goal,o]=ch5Observe(r.task,0);x=[r.task.q0;r.task.qd0];if r.plant.n>12,x=[x;repmat(r.initialForce,3,1)];end
    [u0,d0,p0]=ch5ReducedControlV4(x,r.initialForce,goal,o,r,0);
    x1=full(r.plant.fd(x,u0,zeros(6,1)));[goal,o]=ch5Observe(r.task,r.c.Ts);
    [u1,d1,p1]=ch5ReducedControlV4(x1,u0,goal,o,r,r.c.Ts);
    row=struct('mode',char(mode),'first_refresh',p0.refresh,'next_refresh',p1.refresh,'first_feasible',d0.feasible, ...
        'next_feasible',d1.feasible,'first_cost',p0.cost,'next_cost',p1.cost, ...
        'accepted',p0.refresh&&~p1.refresh&&all(isfinite([u0;u1]))&&isfinite(p0.cost)&&isfinite(p1.cost));
    report.rows{end+1}=row;fprintf('Two-step prior contract %s accepted=%d\n',mode,row.accepted);
end
report.accepted=all(cellfun(@(s)s.accepted,report.rows));
save(fullfile(directory,'audit.mat'),'report','-v7');
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.accepted,'ch5:PriorContract','Consecutive refresh and intermediate execution calls must remain valid.');
end
