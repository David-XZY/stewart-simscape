function report=ch5CheckMovingPrediction()
[c,root]=ch5Init();[m,s,ad,strict]=ch5Physics(c);
r=struct('c',c,'physical',m,'scene',s,'ad',ad,'strict',strict,'mode',"ideal-force",'collision',ch5CollisionModel(m,s,c));
q=m.qHome+[0;0;.15;zeros(3,1)];v=zeros(6,1);o=zeros(22,c.max_obstacles);
o(7:15,:)=repmat(reshape(eye(3),9,1),1,c.max_obstacles);
o(1:3,1)=[3;.4;1.2];o(4:6,1)=[.03;.06;.04];o(16:18,1)=[.01;-.005;.002];o(22,1)=1;
duration=c.H*c.Ts;plan=ch5Ihsid(q,v,q,o,r,duration);assert(plan.accepted,'ch5:MovingPredictionPlanner','Simple CV planner check failed.');
report=struct('version',c.version,'passed',false);
report.baseline_error=check(plan.obstacle_prediction_centers,plan.obstacle_prediction_times,o,c.max_obstacles);
r.plant=ch5PlantModel(m,s,ch5ActuatorProfile(r.mode,c,m),struct('Ts',c.Ts,'substeps',8,'Np',c.H,'integralGridTs',0));
optimizer=ch5PredictiveOptimizer(r,1);[d,G]=ad.dynamics(q,v);u=full(G)\(-full(d));
parameter=[[q;v];u;q;o(:);zeros(6*c.H,1)];[centers,times]=optimizer.obstaclePrediction(parameter);
report.diffusion_error=check(full(centers),full(times),o,c.max_obstacles);
shift=.37;shifted=o;shifted(1:3,:)=o(1:3,:)+shift*o(16:18,:);
parameter2=[[q;v];u;q;shifted(:);zeros(6*c.H,1)];[translated,~]=optimizer.obstaclePrediction(parameter2);
expected=repmat(shift*o(16:18,:),1,numel(full(times)));
report.time_translation_error=max(abs(full(translated)-full(centers)-expected),[],'all');
assert(max([report.baseline_error,report.diffusion_error,report.time_translation_error])<1e-10);
report.passed=true;save(fullfile(root,'chapter5','artifacts','validation','moving_prediction.mat'),'report');
fprintf('CV prediction errors: baseline %.3g, diffusion %.3g, time translation %.3g\n',report.baseline_error,report.diffusion_error,report.time_translation_error);
end
function error=check(centers,times,o,no)
error=0;
for k=1:numel(times)
    actual=centers(:,(k-1)*no+(1:no));expected=o(1:3,:)+times(k)*o(16:18,:);
    error=max(error,max(abs(actual-expected),[],'all'));
end
assert(abs(times(end)-1.2)<1e-12,'ch5:PredictionTime','Unexpected horizon.');
end
