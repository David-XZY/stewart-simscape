function report=ch5SpeedForceRecoveryAuditV71()
% Frozen failed-refresh states compare a retained numerical iterate with a
% fresh actuator-aware braking initializer. No failed prefix is training data.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'recovery_audit_v71.json')),'ch5:ExistingAttempt','Preserve recovery audit.');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
cases={'fault_v68',1,[1.1,1.5,2,2.9];'full_v68',4,[9.7,10.5,13,14.5]};
report=struct('completed',false,'formal',false,'pid',feature('getpid'),'rows',{{}});
for ci=1:size(cases,1)
    index=cases{ci,2};d=load(fullfile(directory,cases{ci,1},sprintf('%03d_nonideal_force.mat',index)),'records','predictions');
    r=ch5ReducedPrepareV48(index,"nonideal-force");
    r.reduced=struct('velocity_weight',1,'factor_derivatives',true,'reuse_sensitivities',false, ...
        'linearization_collision_reserve',.0002,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
    r.force_weights=load(fullfile(directory,'temporal_v64','seed_5101','weights.mat'));k=ch5ForceKernelsV64(r);brake=ch5ReducedBrakeSequence(r);
    for time=cases{ci,3}
        [~,ri]=min(abs(cellfun(@(z)z.time,d.records)-time));z=d.records{ri};p=d.predictions{ri}.prediction;
        old=p.candidates{1};x=z.x;previous=z.previous_u;goal=z.goal;o=z.observed;
        stream=RandStream('mt19937ar','Seed',71000+100*ci+ri);noise=randn(stream,360,4);
        [prior,~,~]=ch5ForceSampleV65(ch5Context(x,previous,goal,o,r.mode),previous,r.force_weights,noise);
        for reset=[false,true]
            ch5ForceTerminalV62();
            if reset,U=full(brake(x,previous));qs=[];else,U=old.U;qs=old.artificial_pose;end
            [before,initial]=ch5ForceAuditV62(x,previous,U,goal,o,r,k,qs);
            timer=tic;[proposals,poses,solves]=ch5ForceStepV66(x,previous,U,goal,o,prior,r,k,time,initial.artificial_pose);seconds=toc(timer);
            accepted=false(1,4);checks=cell(1,4);lines=-ones(1,4);
            for b=1:4
                if ~solves{b}.finite,continue;end
                for line=0:3
                    a=2^(-line);trial=U+a*(proposals(:,:,b)-U);q=initial.artificial_pose+a*(poses(:,b)-initial.artificial_pose);
                    [~,check,~,~]=ch5ForceRetractV62(x,previous,trial,goal,o,r,k,q);
                    checks{b}=check;lines(b)=line;
                    if check.accepted,accepted(b)=true;break;end
                end
            end
            row=struct('source',cases{ci,1},'task',index,'time',time,'reset_to_current_brake',reset,'initializer_check',before, ...
                'proposal_accepted',accepted,'proposal_checks',{checks},'lines',lines,'solves',{solves},'optimization_seconds',seconds);
            report.rows{end+1}=row;writeJSON(fullfile(directory,'recovery_audit_v71.json'),report);
            fprintf('V71 recovery task%d t%.1f reset%d initial%d proposals%d/4\n',index,time,reset,before.accepted,sum(accepted));
        end
    end
end
report.completed=true;writeJSON(fullfile(directory,'recovery_audit_v71.json'),report);save(fullfile(directory,'recovery_audit_v71.mat'),'report','-v7');
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
