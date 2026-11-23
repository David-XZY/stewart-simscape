function report=ch5SpeedThreadAudit()
% Independent stage evaluations only: do not parallelize the state recursion.
% CasADi ThreadMap owns separate work memory for each evaluation thread.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','thread_audit_v1');
assert(~isfile(fullfile(directory,'summary.json')),'ch5:ExistingAudit','Preserve the existing thread audit instead of overwriting it.');
if ~isfolder(directory),mkdir(directory);end
report=struct('rows',{{}},'identity',ch5ReducedIdentity(),'pid',feature('getpid'), ...
    'documentation','https://web.casadi.org/docs/#map','accepted',true);
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepare(4,mode);r.reduced=struct('velocity_weight',1,'linearization_collision_reserve',.0002);
    k=ch5ReducedKernels(r);saved=load(fullfile(root,'chapter5','artifacts','reduced_probe','closed_tapered_v9', ...
        sprintf('004_%s.mat',strrep(mode,'-','_'))),'records');
    for threads=[2,4,8]
        timer=tic;parallel=k;parallel.stages=k.stage.map(r.c.H,'thread',threads);parallel.mechanics=k.mech.map(2*r.c.H,'thread',threads);buildSeconds=toc(timer);
        for index=[26,300]
            sample=saved.records{index};brake=ch5ReducedBrakeSequence(r);U=full(brake(sample.x,sample.previous_u));nodes=full(k.nodes(sample.x,U));
            states=reshape([nodes(:,1:r.c.H);nodes(:,2:end)],r.plant.n,2*r.c.H);commands=repelem(U,1,2);
            [serialStage,~]=evaluate(k.stages,nodes(:,1:r.c.H),U);[threadStage,~]=evaluate(parallel.stages,nodes(:,1:r.c.H),U);
            [serialMech,~]=evaluate(k.mechanics,states,commands);[threadMech,~]=evaluate(parallel.mechanics,states,commands);
            errors=[difference(serialStage,threadStage),difference(serialMech,threadMech)];
            serialSeconds=zeros(2,5);threadSeconds=serialSeconds;
            for repeat=1:5
                if mod(repeat,2)
                    [~,serialSeconds(1,repeat)]=evaluate(k.stages,nodes(:,1:r.c.H),U);
                    [~,threadSeconds(1,repeat)]=evaluate(parallel.stages,nodes(:,1:r.c.H),U);
                    [~,serialSeconds(2,repeat)]=evaluate(k.mechanics,states,commands);
                    [~,threadSeconds(2,repeat)]=evaluate(parallel.mechanics,states,commands);
                else
                    [~,threadSeconds(1,repeat)]=evaluate(parallel.stages,nodes(:,1:r.c.H),U);
                    [~,serialSeconds(1,repeat)]=evaluate(k.stages,nodes(:,1:r.c.H),U);
                    [~,threadSeconds(2,repeat)]=evaluate(parallel.mechanics,states,commands);
                    [~,serialSeconds(2,repeat)]=evaluate(k.mechanics,states,commands);
                end
            end
            [goal,o]=ch5Observe(r.task,sample.time);prior=ones(size(U))*16;
            [~,serialQP]=ch5ReducedStepV9(sample.x,sample.previous_u,U,goal,o,prior,r,k,true);
            [~,threadQP]=ch5ReducedStepV9(sample.x,sample.previous_u,U,goal,o,prior,r,parallel,true);
            matrixErrors=cellfun(@(name)norm(serialQP.(name)-threadQP.(name),inf),{'Q','f','A','b','E','e','M','nodes'});
            row=struct('mode',char(mode),'threads',threads,'sample',index,'build_seconds',buildSeconds, ...
                'output_errors',errors,'matrix_errors',matrixErrors,'serial_seconds',serialSeconds,'thread_seconds',threadSeconds, ...
                'accepted',all(isfinite([errors,matrixErrors]))&&max([errors,matrixErrors])<1e-10);
            report.rows{end+1}=row;report.accepted=report.accepted&&row.accepted;
            fprintf('Thread audit %s threads%d sample%d accepted=%d stage %.2f->%.2fms mechanics %.2f->%.2fms\n', ...
                mode,threads,index,row.accepted,1000*median(serialSeconds(1,:)),1000*median(threadSeconds(1,:)),1000*median(serialSeconds(2,:)),1000*median(threadSeconds(2,:)));
            save(fullfile(directory,'audit.mat'),'report','-v7');
        end
    end
end
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(report.accepted,'ch5:ParallelMismatch','Parallel numerical outputs and condensed matrices must match serial evaluation.');
end

function [outputs,seconds]=evaluate(f,x,u)
timer=tic;[a,b,c]=f(x,u);outputs={full(a),full(b),full(c)};seconds=toc(timer);
end

function errors=difference(a,b)
errors=cellfun(@(x,y)norm(x-y,inf),a,b);
end
