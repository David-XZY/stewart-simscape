function report=ch5CheckIntervalConstraints()
% Verify encapsulation against the original shared full physical trace.
[c,root]=ch5Init();[m,s,ad,strict]=ch5Physics(c);import casadi.*
r=struct('c',c,'physical',m,'scene',s,'ad',ad,'strict',strict,'mode',"nonideal-force", ...
    'collision',ch5CollisionModel(m,s,c));
r.plant=ch5PlantModel(m,s,ch5ActuatorProfile(r.mode,c,m),struct('Ts',c.Ts,'substeps',8,'Np',c.H,'integralGridTs',0));
active=[1,2,3];map=ch5IntervalConstraints(r,active);q=m.qHome+[0;0;.15;zeros(3,1)];
[d,G]=ad.dynamics(q,zeros(6,1));equilibrium=full(G)\(-full(d));
o=zeros(22,c.max_obstacles);o(7:15,:)=repmat(reshape(eye(3),9,1),1,c.max_obstacles);
o(1:3,active)=[2,3,4;.5,.7,.9;1,1.2,1.4];o(4:6,active)=.1;o(16:18,active)=.01;o(22,active)=1;
xx=MX.sym('check_interval_x',r.plant.n);uu=MX.sym('check_interval_u',6);value=map(xx,uu,o,.37);
derivative=Function('ch5_interval_input_derivative',{xx,uu},{jacobian(value,uu)});
primal=0;inputError=0;generator=RandStream('mt19937ar','Seed',7933);
for k=1:6
    x=[q;zeros(6,1);repmat(equilibrium,3,1)];u=equilibrium;
    if k>1,x(7:12)=.005*randn(generator,6,1);u=u+30*randn(generator,6,1);end
    [~,trace]=r.plant.fd(x,u,zeros(6,1));expected=[];
    for j=1:numel(r.plant.traceTime)
        ot=o;ot(1:3,:)=o(1:3,:)+(.37+r.plant.traceTime(j))*o(16:18,:);
        physical=full(r.plant.mechanicalValues(trace(:,j),u));gap=full(r.collision.value(trace(1:6,j),ot));
        expected=[expected;physical(1:19);gap(active)]; %#ok<AGROW>
    end
    primal=max(primal,max(abs(full(map(x,u,o,.37))-expected)));
    B=full(derivative(x,u));h=.001;
    for j=1:6
        perturb=zeros(6,1);perturb(j)=h;
        numerical=(full(map(x,u+perturb,o,.37))-full(map(x,u-perturb,o,.37)))/(2*h);
        inputError=max(inputError,max(abs(B(:,j)-numerical)));
    end
end
report=struct('control_revision',c.control_revision,'primal_error',primal,'input_jacobian_error',inputError, ...
    'internal_points',numel(r.plant.traceTime),'cases',6,'passed',primal<1e-8&&inputError<1e-4);
save(fullfile(root,'chapter5','artifacts','validation','interval_constraints.mat'),'report');disp(report);assert(report.passed);
end
