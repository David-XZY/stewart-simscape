function report=ch5SpeedReducedPointSparse()
[c,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','point_sparse_v1');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'pid',feature('getpid'));saved=load(fullfile(root,'chapter5','artifacts','tasks','development.mat'),'tasks');
for mode=["ideal-force","nonideal-force"]
    [physical,scene,ad,strict]=ch5Physics(c);profile=ch5ActuatorProfile(mode,c,physical);
    r=struct('c',c,'mode',mode,'physical',physical,'scene',scene,'ad',ad,'strict',strict, ...
        'collision',ch5CollisionModel(physical,scene,c),'plant',ch5PlantModel(physical,scene,profile,struct('Ts',c.Ts,'substeps',8,'Np',c.H,'integralGridTs',0)),'task',saved.tasks{1});
    r.c.version=[c.version,'-speed-reduced-v1'];k=ch5ReducedKernels(r);ch5ReducedFeedbackRollout(r);
    [goal,o]=ch5Observe(r.task,0);x=[r.task.q0;r.task.qd0];previous=full(k.equilibrium(x(1:6)));
    if r.plant.n>12,x=[x;repmat(previous,3,1)];end
    [y,A,B]=k.stage(x,previous);y=full(y);A=full(A);B=full(B);finiteB=zeros(size(B));
    for j=1:6,du=zeros(6,1);du(j)=.001;yp=k.stage(x,previous+du);ym=k.stage(x,previous-du);finiteB(:,j)=(full(yp)-full(ym))/.002;end
    fprintf('Reduced stage %s finite derivative error %.4g\n',mode,norm(B-finiteB,inf));
    gain=ch5ReducedFeedbackGain(x,r,k);
    report.derivatives.(strrep(mode,'-','_'))=struct('input_jacobian_error',norm(B-finiteB,inf), ...
        'primal_error',norm(y-full(r.plant.fd(x,previous,zeros(6,1))),inf), ...
        'open_loop_spectral_radius',max(abs(eig(A))),'feedback_spectral_radius',max(abs(eig(A-B*gain))));
    U=repmat(previous,1,r.c.H);prior=zeros(size(U));
    for iteration=1:3
        timer=tic;[check,trajectory,cost]=ch5ReducedAudit(x,previous,U,goal,o,r,k,prior);
        [proposed,info]=ch5ReducedStepSparse(x,previous,U,goal,o,prior,r,k);retained=false;tests={};
        for line=0:5
            trial=U+2^(-line)*(proposed-U);[trial,repair]=ch5ReducedTerminal(x,trial,r,k);
            [trialCheck,trialTrajectory,trialCost]=ch5ReducedAudit(x,previous,trial,goal,o,r,k,prior);
            tests{end+1}=struct('line',line,'cost',trialCost,'check',trialCheck,'repair',repair);
            if trialCheck.accepted&&trialCost<cost,U=trial;check=trialCheck;trajectory=trialTrajectory;cost=trialCost;retained=true;break;end
        end
        row=struct('mode',char(mode),'iteration',iteration,'seconds',toc(timer),'solver',info,'check',check, ...
            'retained',retained,'cost',cost,'terminal_position_error',norm(trajectory.X(1:3,end)-goal(1:3)),'tests',{tests});
        report.rows{end+1}=row;
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
        fprintf('Reduced sparse feedback OSQP %s %d: %.4fs solver %.4fs linearization %.4fs retained=%d endpoint %.6fm\n',mode,iteration,row.seconds,info.solver_seconds,info.linearization_seconds,retained,row.terminal_position_error);
    end
    save(fullfile(directory,strrep(mode,'-','_')+'.mat'),'r','x','previous','U','report','-v7');
end
end
