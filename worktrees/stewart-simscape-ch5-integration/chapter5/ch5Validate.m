function report=ch5Validate()
[c,root]=ch5Init();[m,s,ad,strict]=ch5Physics(c);collision=ch5CollisionModel(m,s,c);
report=struct('version',c.version,'matlab',version,'licenses',[license('test','Simulink'),license('test','Simscape'),license('test','SimMechanics'),license('test','Optimization_Toolbox')]);
report.motion_rhs_equivalence_error=0;
for k=1:10
    qtest=m.qHome+[.02*sin(k);.02*cos(k);.12;deg2rad(k);-.04;.06];vtest=.01*sin((1:6)'+k);
    forceTest=260+20*cos((1:6)'+k);[dtest,Gtest]=ad.dynamics(qtest,vtest);
    exact=full(ad.motion(qtest,vtest,forceTest,zeros(6,1)));
    report.motion_rhs_equivalence_error=max(report.motion_rhs_equivalence_error,norm(exact-[vtest;full(dtest+Gtest*forceTest)],inf));
end
assert(report.motion_rhs_equivalence_error<1e-9,'ch5:PhysicsEquivalence','Closed-form rigid dynamics mismatch.');
frame=[.2;-.1;1.1;.1;.2;-.3];relative=[.02;.04;-.03;-.2;.1;.15];velocity=[.01;-.02;.03;.01;.03;-.02];
[q,v]=ch5AbsoluteState(relative,velocity,frame);expected=ch5Rotation(frame(4:6))*ch5Rotation(relative(4:6));
report.rotation_composition_error=norm(ch5Rotation(q(4:6))-expected,'fro');
report.velocity_error=norm(rpyRateMapZYX(q(4:6))*v(4:6)-ch5Rotation(frame(4:6))*velocity(4:6));
assert(report.rotation_composition_error<1e-12&&report.velocity_error<1e-12);
du=reshape(1:360,6,60)*.001;previous=(1:6)';U=previous+cumsum(du,2);
assert(norm(U(:,1)-previous-du(:,1))<1e-12);assert(norm(U(:,end)-previous-sum(du,2))<1e-12);
task=struct('id','interface_nonzero_pose','seed',4501,'index',1,'q0',m.qHome+[.02;-.02;.15;deg2rad(3);deg2rad(-4);deg2rad(7)], ...
    'qd0',zeros(6,1),'goals',m.qHome,'goal_times',0,'obstacles',zeros(22,c.max_obstacles), ...
    'motion_velocity',zeros(3,c.max_obstacles),'visible_times',zeros(1,c.max_obstacles),'turn_rate',zeros(1,c.max_obstacles));
task.obstacles(7:15,:)=repmat(reshape(eye(3),9,1),1,c.max_obstacles);task.goals=task.q0;
for mode=["ideal-force","nonideal-force"]
    p=ch5ActuatorProfile(mode,c,m);plant=ch5PlantModel(m,s,p,struct('Ts',c.Ts,'substeps',8,'Np',c.H,'integralGridTs',0));
    [drift,G]=ad.dynamics(task.q0,zeros(6,1));force=full(G)\(-full(drift));x=[task.q0;zeros(6,1)];
    if mode=="nonideal-force",x=[x;repmat(force,3,1)];end
    xnext=full(plant.fd(x,force,zeros(6,1)));residual=norm(xnext-x);
    [~,B]=plant.derivative(x,force);B=full(B);numerical=zeros(size(B));eps=1e-3;
    for j=1:6,up=force;um=force;up(j)=up(j)+eps;um(j)=um(j)-eps;numerical(:,j)=(full(plant.fd(x,up,zeros(6,1)))-full(plant.fd(x,um,zeros(6,1))))/(2*eps);end
    % At a rate-limiter kink its directional derivative is nonsmooth; test
    % the mechanical component, rather than a fictitious unique actuator slope.
    error=norm(B(1:12,:)-numerical(1:12,:),'fro');assert(residual<1e-8&&error<1e-5);
    label=strrep(char(mode),'-','_');report.(label)=struct('equilibrium_residual',residual,'mechanical_derivative_error',error,'state_dimension',plant.n);
    short=c;short.episode_seconds=.2;
    file=fullfile(root,'chapter5','artifacts','validation',[label,'_nonzero_pose.mat']);
    result=ch5RunEpisode(task,mode,'hold',0,short,file);report.(label).simscape=result;
    assert(strcmp(result.status,'completed'),'ch5:InterfaceRunFailed','Simscape interface validation failed.');
    report.(label).static_position_error=result.metrics.final_position;
    report.(label).static_rotation_error=result.metrics.final_rotation;
    assert(result.metrics.final_position<1e-4&&result.metrics.final_rotation<deg2rad(.05),'ch5:StaticMismatch','Absolute pose or equilibrium mismatch.');
    motionTask=task;motionTask.id='short_motion';motionTask.probe_force=[5;-3;4;-2;3;1];
    file=fullfile(root,'chapter5','artifacts','validation',[label,'_short_motion.mat']);
    moving=ch5RunEpisode(motionTask,mode,'hold',0,short,file);assert(strcmp(moving.status,'completed'));
    predicted=x;for k=1:round(short.episode_seconds/c.Ts),predicted=full(plant.fd(predicted,force+motionTask.probe_force,zeros(6,1)));end
    sample=load(file,'audit');lastPose=sample.audit.absolute_pose(end,:)';
    report.(label).motion_prediction_position_error=norm(predicted(1:3)-lastPose(1:3));
    report.(label).motion_prediction_rotation_error=norm(ch5Rotation(predicted(4:6))-ch5Rotation(lastPose(4:6)),'fro');
    assert(report.(label).motion_prediction_position_error<1e-4&&report.(label).motion_prediction_rotation_error<1e-3, ...
        'ch5:MotionMismatch','Short-motion Simscape/actuator prediction mismatch.');
end
o=task.obstacles;o(:,1)=[1;0;1.2;.1;.1;.2;reshape(eye(3),9,1);.01;-.02;.03;zeros(3,1);1];
[~,~,ht]=collision.cbf(task.q0,zeros(6,1),o);op=o;om=o;delta=1e-4;op(1:3,1)=op(1:3,1)+delta*o(16:18,1);om(1:3,1)=om(1:3,1)-delta*o(16:18,1);
numeric=(full(collision.value(task.q0,op))-full(collision.value(task.q0,om)))/(2*delta);
report.moving_collision_time_derivative_error=norm(full(ht)-numeric);assert(report.moving_collision_time_derivative_error<1e-6);
report.passed=true;save(fullfile(root,'chapter5','artifacts','validation','interface.mat'),'report');
fid=fopen(fullfile(root,'chapter5','artifacts','validation','interface.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('Interface validation passed.\n');
end
