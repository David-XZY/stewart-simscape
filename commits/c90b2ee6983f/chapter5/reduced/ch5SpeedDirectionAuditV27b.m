function report=ch5SpeedDirectionAuditV27b()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'direction_audit_v27b');
assert(~isfolder(directory),'ch5:ExistingAudit','Preserve previous attempts.');mkdir(directory);
data=load(fullfile(base,'projection_audit_v25','audit.mat'),'report');original=load(fullfile(base,'failure_replay_v23','audit.mat'),'report');
report=struct('pid',feature('getpid'),'rows',{{}},'completed',false,'error','');
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepare(4,mode);r.reduced=struct('reuse_sensitivities',false,'velocity_weight',1,'factor_derivatives',false, ...
            'feasibility_projection',true,'trust_radius',.1,'qp_row_scaling',true,'linearization_collision_reserve',.0002);k=ch5ReducedKernelsV4(r);
        idx=find(cellfun(@(v)strcmp(v.mode,char(mode))&&~v.reuse_enabled,data.report.rows),1);U=data.report.rows{idx}.trials{1}.U;
        oi=find(cellfun(@(v)strcmp(v.mode,char(mode)),original.report.rows),1);s=original.report.rows{oi}.replay;
        [~,linear]=ch5ReducedStepV27(s.x,s.previous,U,s.goal,s.observed,s.prior,r,k,.6,true);[delta,solver]=ch5ReducedSolveV24(linear,r);
        observed=s.observed(:,1);observed(1:3)=observed(1:3)+observed(16:18)*1.2;
        [gap,gradient]=k.collision(linear.nodes(1:6,end),observed);gap=full(gap);gradient=full(gradient);
        predictedPose=linear.nodes(1:6,end)+linear.state_sensitivity(1:6,:,end)*delta;
        row=struct('mode',char(mode),'solver',solver,'delta_norm',norm(delta,inf),'base_gap',gap,'predicted_gap',gap+gradient*(predictedPose-linear.nodes(1:6,end)), ...
            'predicted_pose_change',predictedPose-linear.nodes(1:6,end),'steps',{{}},'linear',linear,'delta',delta);
        feedback=ch5ReducedFeedbackRollout(r);
        for alpha=[1,.1,.01,.001]
            [proposal,nodes]=feedback(s.x,linear.nodes,U,linear.gain,reshape(alpha*delta,6,r.c.H/5));proposal=full(proposal);nodes=full(nodes);
            beforeGap=full(k.collision(nodes(1:6,end),observed));
            [after,check,trajectory,~,audit]=ch5ReducedAuditV23(s.x,s.previous,proposal,s.goal,s.observed,r,k,s.prior);
            afterGap=full(k.collision(trajectory.X(1:6,end),observed));
            endpointError=nodes(:,end)-linear.nodes(:,end)-alpha*linear.state_sensitivity(:,:,end)*delta;
            row.steps{end+1}=struct('alpha',alpha,'before_gap',beforeGap,'after_gap',afterGap,'check',check,'endpoint_linearization_error',endpointError, ...
                'actual_pose_change',nodes(1:6,end)-linear.nodes(1:6,end),'repair_pose_change',trajectory.X(1:6,end)-nodes(1:6,end),'audit',audit);
            fprintf('Direction %s a%.3g base %.6fmm predicted %.6fmm before %.6fmm after %.6fmm poseerr %.4g\n',mode,alpha,1000*gap, ...
                1000*(gap+alpha*gradient*(predictedPose-linear.nodes(1:6,end))),1000*beforeGap,1000*afterGap,norm(endpointError(1:6),inf));
        end
        report.rows{end+1}=row;write(directory,report);
    end
    report.completed=true;
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write(directory,report);rethrow(ex);end
write(directory,report);
end
function write(directory,report)
save(fullfile(directory,'audit.mat'),'report','-v7');summary=report;
for j=1:numel(summary.rows),summary.rows{j}=rmfield(summary.rows{j},{'linear','delta'});end
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(summary));fclose(fid);
end
