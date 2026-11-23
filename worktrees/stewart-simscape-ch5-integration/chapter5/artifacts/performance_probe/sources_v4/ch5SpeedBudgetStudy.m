function report=ch5SpeedBudgetStudy()
[~,root]=ch5Init();out=fullfile(root,'chapter5','artifacts','performance_probe','budget_v4');
if ~isfolder(out),mkdir(out);end
report=struct('pid',feature('getpid'),'code_identity',ch5SpeedIdentity(),'rows',{{}}, ...
    'scope','Paired fixed-state solver budgets; no formal outcomes or real-time deployment');
journal=load(fullfile(root,'chapter5','artifacts','performance_probe','closed_loop','001_ideal_force_journal','chunk_00001.mat'));
perturbed=journal.newRecords{2}.x;
for mode=["ideal-force","nonideal-force"]
    for polish=[3,10,30]
        label=sprintf('braking_one_sqp_%d_polish',polish);r=ch5SpeedPrepare(1,mode,label,polish);
        [goal,o]=ch5Observe(r.task,0);x=[r.task.q0;r.task.qd0];
        if mode=="nonideal-force",x=[x;repmat(r.initialForce,3,1)];end
        build=tic;preview=ch5NativePreview(r);ch5NativeNodes(r);ch5PredictiveOptimizer(r,[]);ch5PredictiveOptimizer(r,[],true);
        [~,~,trace]=preview(x,repmat(r.initialForce,1,r.c.H));ch5NativeCollisionAudit(full(trace),o,r);buildSeconds=toc(build);
        for sample=1:2
            if sample==2,x(1:12)=perturbed;end
            for name=["ch5SpeedWarm","ch5SpeedIterate"]
                if isappdata(0,name),rmappdata(0,name);end
            end
            timer=tic;[u,diagnostic,prediction]=ch5SpeedDiffusionControl(x,r.initialForce,goal,o,r,(sample-1)*.02);seconds=toc(timer);
            trajectory=prediction.candidates{1};terminal=norm(trajectory.X(1:3,end)-goal(1:3));initial=norm(x(1:3)-goal(1:3));
            row=struct('mode',mode,'polish_iterations',polish,'sample',sample,'seconds',seconds, ...
                'build_seconds',buildSeconds,'deadline_met',seconds<=.1,'fallback',diagnostic.usedFallback, ...
                'initial_position_error',initial,'terminal_position_error',terminal,'predicted_improvement',initial-terminal, ...
                'state',x,'u',u,'prediction',prediction);
            report.rows{end+1}=row;save(fullfile(out,'study.mat'),'report','-v7');
            fprintf('Budget %s NLP%d state%d: %.4fs fallback=%d terminal %.6fm improvement %.6fm\n',mode,polish,sample,seconds,diagnostic.usedFallback,terminal,initial-terminal);
        end
        clear ch5PredictiveOptimizer ch5NativePreview ch5NativeNodes
    end
end
report.completed=true;save(fullfile(out,'study.mat'),'report','-v7');
fid=fopen(fullfile(out,'study.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
