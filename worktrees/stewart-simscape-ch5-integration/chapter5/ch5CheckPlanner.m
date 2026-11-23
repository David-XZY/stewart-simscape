function report=ch5CheckPlanner()
[c,root]=ch5Init();[m,s,ad,strict]=ch5Physics(c);tasks=load(fullfile(root,'chapter5','artifacts','tasks','train.mat'));
r=struct('c',c,'physical',m,'scene',s,'ad',ad,'strict',strict,'collision',ch5CollisionModel(m,s,c));
task=tasks.tasks{1};[goal,o]=ch5Observe(task,0);report=struct('version',c.version,'durations',[6,5.8],'accepted',false(1,2));
for j=1:2
    plan=ch5Ihsid(task.q0,task.qd0,goal,o,r,report.durations(j));
    report.accepted(j)=plan.accepted;report.plans{j}=plan;
    assert(plan.accepted,'ch5:PlannerSmoke','Independent collocation interface failed before teacher collection.');
end
report.passed=true;save(fullfile(root,'chapter5','artifacts','validation','planner.mat'),'report');
end
