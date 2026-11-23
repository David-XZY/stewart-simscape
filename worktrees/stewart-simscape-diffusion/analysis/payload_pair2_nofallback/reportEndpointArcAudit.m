function reportEndpointArcAudit()
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
base=fullfile(root,'results','chapter3','payload_pair2_nofallback_v1');
out=fullfile(base,'candidate_joint_cases_v1','71c93079c72cff94b9929aed');
d=load(fullfile(out,'plot_data.mat'));
saved=load(fullfile(base,'formal','calls',d.metadata.runId,'trajectory.mat'));
task=saved.task; result=saved.result;
c=pgConfig('payload_pair2_nofallback_v1/candidate_joint_cases_v1/arc_audit'); c.controlCount=16; c.denseStep=.0025;
raw=struct('kind','spline','T',task.T,'cp',d.candidateCP(:,:,12)); final=result.bundle;
t=linspace(0,task.T,2401); direction=task.qGoal(1:3)-task.q0(1:3); direction=direction/norm(direction);
rawStats=describe(raw); finalStats=describe(final);
[value,~,groups]=pgPenalty(reshape(final.cp(:,4:13),[],1),task,c);
trials=cell(4,1); baseline=bdBaseline(task,16);
for k=1:4
    factor=k/4; candidate=final;
    candidate.cp(1:3,10:13)=(1-factor)*final.cp(1:3,10:13)+factor*baseline(1:3,10:13);
    gate=pgValidate(candidate,task,c); stats=describe(candidate);
    trials{k}=struct('blendFraction',factor,'passed',gate.passed,'reason',gate.reason,'stats',stats);
end
trials{5}=struct(); candidate=final;
for j=10:13
    excess=direction.'*(candidate.cp(1:3,j)-task.qGoal(1:3));
    candidate.cp(1:3,j)=candidate.cp(1:3,j)-max(0,excess)*direction;
end
gate=pgValidate(candidate,task,c); stats=describe(candidate);
trials{5}=struct('type','clamp_final_free_control_forward_overshoot','passed',gate.passed,'reason',gate.reason,'stats',stats);
report=struct('runId',d.metadata.runId,'raw',rawStats,'final',finalStats,'coarsePenalty',value, ...
    'coarsePenaltyGroups',groups,'counterfactuals',{trials}, ...
    'scope','Offline counterfactual checks only; original experiment and returned trajectory unchanged. Last four free position controls moved; endpoint, timing and orientation controls unchanged.');
dpWriteJson(fullfile(out,'endpoint_arc_audit.json'),report);
disp(jsonencode(report));
 function stats=describe(b)
    tr=bdReconstruct(b,t); p=tr.q(1:3,:); goal=task.qGoal(1:3); dist=sqrt(sum((p-goal).^2,1));
    forward=direction.'*(p-goal); [peak,index]=max(forward); tail=find(t>=.8*task.T);
    steps=sqrt(sum(diff(p,1,2).^2,1));
    [tailPeak,ti]=max(dist(tail));
    stats=struct('lengthM',sum(steps),'maxForwardOvershootM',peak,'maxForwardOvershootTime',t(index), ...
        'positionAtOvershoot',p(:,index),'rpyDegAtOvershoot',tr.q(4:6,index)*180/pi, ...
        'tail20PercentLengthM',sum(steps(tail(1):end)),'tail20PercentChordM',dist(tail(1)), ...
        'tailMaxGoalDistanceM',tailPeak,'tailMaxGoalDistanceTime',t(tail(ti)), ...
        'lastFourFreePositionControls',b.cp(1:3,10:13),'lastFourControlForwardM',direction.'*(b.cp(1:3,10:13)-goal));
 end
end
