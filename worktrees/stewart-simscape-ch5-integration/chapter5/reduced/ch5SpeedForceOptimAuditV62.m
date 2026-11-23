function report=ch5SpeedForceOptimAuditV62()
% Pair exact implementation changes on the same frozen 96 V60 candidates.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62');
assert(~isfile(fullfile(directory,'optimization_audit.mat')),'ch5:ExistingAttempt','Preserve earlier optimization audit.');
gate=load(fullfile(directory,'kernel_audit.mat'),'report');assert(gate.report.accepted);
oldDirectory=fullfile(root,'chapter5','artifacts','control_force_v57','pilot');
s=load(fullfile(oldDirectory,'selection.mat'));data=load(fullfile(oldDirectory,'candidates_5101.mat'));
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'completed',false,'accepted',false,'rows',{{}},'groups',{{}},'formal',false);
for mode=["nonideal-force","ideal-force"]
    r=ch5ReducedPrepareV48(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false,'reuse_sensitivities',false, ...
        'linearization_collision_reserve',.0002,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
    original=ch5ReducedKernelsV4(r);fastRuntime=r;fastRuntime.reduced.factor_derivatives=true;k=ch5ReducedKernelsV4(fastRuntime);
    for index=unique(data.selection_index)
        c=double(s.context(:,index));if (c(219)>.5)~=strcmp(mode,'nonideal-force'),continue;end
        ch5ForceTerminalV62();x=c(1:r.plant.n);previous=c(31:36);goal=c(37:42);o=reshape(c(43:218),22,8);
        ids=find(data.selection_index==index);priors=reshape(double(data.residual(:,ids)),6,60,4)+previous;
        brake=ch5ReducedBrakeSequence(r);inc=full(brake(x,previous));[incCheck,incTraj]=ch5ForceAuditV58(x,previous,inc,goal,o,r,original,[]);
        if ~incCheck.accepted,[inc,~]=ch5ReducedTerminalV23(x,inc,r,original,incTraj.X);[incCheck,incTraj]=ch5ForceAuditV58(x,previous,inc,goal,o,r,original,[]);end
        timer=tic;[oldU,oldQ,oldSolves]=ch5ForceStepV59(x,previous,inc,goal,o,priors,r,original,0,incTraj.artificial_pose);oldSeconds=toc(timer);
        timer=tic;[newU,newQ,newSolves]=ch5ForceStepV62(x,previous,inc,goal,o,priors,fastRuntime,k,0,incTraj.artificial_pose);newSeconds=toc(timer);
        proposalError=max(abs(oldU-newU),[],'all');poseError=max(abs(oldQ-newQ),[],'all');
        report.groups{end+1}=struct('index',index,'mode',char(mode),'original_seconds',oldSeconds,'optimized_seconds',newSeconds, ...
            'proposal_error_N',proposalError,'pose_error',poseError,'new_phases',newSolves{1}.linearization_phases);
        for b=1:4
            [rawOld,~]=ch5ForceAuditV58(x,previous,priors(:,:,b),goal,o,r,original,[]);
            [rawNew,~]=ch5ForceAuditV62(x,previous,priors(:,:,b),goal,o,fastRuntime,k,[]);
            projected=struct('accepted',false);cost=inf;lineUsed=-1;
            if newSolves{b}.finite
                for line=0:3
                    a=2^(-line);trial=inc+a*(newU(:,:,b)-inc);qs=incTraj.artificial_pose+a*(newQ(:,b)-incTraj.artificial_pose);
                    [~,projected,~,cost]=ch5ForceRetractV62(x,previous,trial,goal,o,fastRuntime,k,qs);
                    lineUsed=line;if projected.accepted,break;end
                end
            end
            report.rows{end+1}=struct('selection_index',index,'candidate',b,'mode',char(mode),'raw_check',rawNew,'raw_acceptance_equal',rawOld.accepted==rawNew.accepted, ...
                'projected_check',projected,'projected_cost',cost,'line',lineUsed,'old_solve',oldSolves{b},'new_solve',newSolves{b});
        end
        writeReport(directory,report);fprintf('V62 group%d %s old%.3f new%.3f proposal%.3gN\n',index,mode,oldSeconds,newSeconds,proposalError);
    end
end
report.completed=true;
report.accepted=all(cellfun(@(v)v.proposal_error_N<1e-4&&v.pose_error<1e-7,report.groups))&& ...
    all(cellfun(@(v)v.raw_acceptance_equal&&v.new_solve.solver_success,report.rows));
writeReport(directory,report);assert(report.accepted,'ch5:OptimizationMismatch','Paired optimized proposal or audit differs.');
end
function writeReport(directory,report)
save(fullfile(directory,'optimization_audit.mat'),'report','-v7');
fid=fopen(fullfile(directory,'optimization_audit.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
