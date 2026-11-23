function report=ch5SpeedReducedPoint()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','point_v1');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'pid',feature('getpid'));
for mode=["ideal-force","nonideal-force"]
    r=ch5SpeedPrepare(1,mode,'reduced-v1',0);timer=tic;k=ch5ReducedKernels(r);buildSeconds=toc(timer);
    [goal,o]=ch5Observe(r.task,0);x=[r.task.q0;r.task.qd0];previous=r.initialForce;
    if r.plant.n>12,x=[x;repmat(previous,3,1)];end
    U=repmat(previous,1,r.c.H);prior=zeros(6,r.c.H);rows={};
    for iteration=1:5
        timer=tic;[check,trajectory,cost]=ch5ReducedAudit(x,previous,U,goal,o,r,k,prior);
        [proposed,info]=ch5ReducedStep(x,previous,U,goal,o,prior,r,k);
        [proposed,repair]=ch5ReducedTerminal(x,proposed,r,k);
        [proposedCheck,proposedTrajectory,proposedCost]=ch5ReducedAudit(x,previous,proposed,goal,o,r,k,prior);
        retained=proposedCheck.accepted&&proposedCost<cost;
        if retained,U=proposed;check=proposedCheck;trajectory=proposedTrajectory;cost=proposedCost;end
        row=struct('mode',char(mode),'iteration',iteration,'seconds',toc(timer),'solver',info,'repair',repair, ...
            'check',check,'proposal_check',proposedCheck,'retained',retained,'cost',cost, ...
            'terminal_position_error',norm(trajectory.X(1:3,end)-goal(1:3)),'build_seconds',buildSeconds);
        rows{end+1}=row;fprintf('Reduced %s %d: %.4fs accepted=%d retained=%d endpoint %.6fm violation %.4g terminal %.4g\n',mode,iteration,row.seconds,check.accepted,retained,row.terminal_position_error,proposedCheck.inequality_violation,proposedCheck.equality_tolerance_ratio);
    end
    file=fullfile(directory,strrep(mode,'-','_')+'.mat');save(file,'r','x','previous','U','rows','-v7');report.rows=[report.rows,rows];
    fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
end
