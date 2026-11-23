function report=ch5SpeedTeacherFailureAuditV83()
% Replay saved observations, not simulations or future obstacle truth.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_failure_v83');
if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'replay.json')),'ch5:ExistingAttempt','Preserve replay evidence.');
source=fullfile(fileparts(directory),'teacher_pilot_v82');summary=jsondecode(fileread(fullfile(source,'summary.json')));
report=struct('completed',false,'formal',false,'pid',feature('getpid'),'rows',{{}},'negative_checks',[]);
for i=1:numel(summary.rows)
    result=summary.rows(i);if result.admitted||result.subtype~=3,continue;end
    mode=string(result.mode);r=ch5ReducedPrepareV48(4,mode);
    d=load(fullfile(source,sprintf('%s_%s.mat',result.task,strrep(result.mode,'-','_'))),'records','plans');
    s=struct('goal',nan(6,1),'plan',struct('accepted',false),'planTime',0);oldFirst=inf;newFirst=inf;rows={};
    for pi=1:numel(d.plans)
        p=d.plans{pi};[~,ri]=min(abs(cellfun(@(z)z.time,d.records)-p.time));z=d.records{ri};
        if p.time>=4&&s.plan.accepted&&isfield(s.plan,'reference_mode')&&strcmp(s.plan.reference_mode,'task')
            [old,a]=ch5BaselineRetainV72(s,z.q,z.qd,z.goal,z.observed,r,p.time);
            [candidate,b]=ch5BaselineRetainV83(s,z.q,z.qd,z.goal,z.observed,r,p.time);
            rows{end+1}=struct('time',p.time,'old_retained',old,'new_retained',candidate,'old_check',a.previous_reference_check,'new_check',b.previous_reference_check); %#ok<AGROW>
            if ~isempty(a.previous_reference_check)&&~a.previous_reference_check.accepted,oldFirst=min(oldFirst,p.time);end
            if ~isempty(b.previous_reference_check)&&~b.previous_reference_check.accepted,newFirst=min(newFirst,p.time);end
            if candidate,example=struct('s',s,'z',z,'r',r,'time',p.time);end
        end
        if strcmp(p.reference_decision,'new_task_plan')
            s.plan=p.plan;s.plan.reference_mode='task';s.planTime=p.time;
        elseif ~isempty(p.backup_plan)&&p.backup_plan.accepted
            s.plan=p.backup_plan;s.plan.reference_mode='backup';s.planTime=p.time;
        elseif strcmp(p.reference_decision,'unverified_braking_after_both_rejections')
            s.plan.accepted=false;
        end
        s.goal=z.goal;
    end
    failed=find(cellfun(@(z)z.diagnostic.usedFallback,d.records),1);first=d.records{failed};
    row=struct('task',result.task,'mode',result.mode,'old_first_hold_rejection',oldFirst,'new_first_hold_rejection',newFirst, ...
        'first_qp_failure',first.time,'new_lead_before_qp_failure',first.time-newFirst,'checks',{rows});
    report.rows{end+1}=row;
    fprintf('V83 replay %s %s old reject%.2f new reject%.2f first QP fail%.2f\n',result.task,mode,oldFirst,newFirst,first.time);
end
assert(numel(report.rows)==6&&all(cellfun(@(z)z.new_lead_before_qp_failure>.2,report.rows)),'ch5:HorizonReplay','Expected earlier warning in every preserved orbit failure.');
s=example.s;z=example.z;r=example.r;goal=z.goal;goal(1)=goal(1)+.002;
[changedGoal,~]=ch5BaselineRetainV83(s,z.q,z.qd,goal,z.observed,r,example.time);
q=z.q;q(1)=q(1)+.005;[drift,~]=ch5BaselineRetainV83(s,q,z.qd,z.goal,z.observed,r,example.time);
o=z.observed;o(:,1)=0;o(1:3,1)=z.goal(1:3);o(4:6,1)=.2;o(7:15,1)=reshape(eye(3),9,1);o(22,1)=1;
[collision,~]=ch5BaselineRetainV83(s,z.q,z.qd,z.goal,o,r,example.time);
report.negative_checks=struct('goal_change_rejected',~changedGoal,'tracking_drift_rejected',~drift,'goal_obstacle_rejected',~collision);
assert(~changedGoal&&~drift&&~collision,'ch5:HorizonReplay','A changed or unsafe reference was retained.');
report.completed=true;writeJSON(fullfile(directory,'replay.json'),report);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
