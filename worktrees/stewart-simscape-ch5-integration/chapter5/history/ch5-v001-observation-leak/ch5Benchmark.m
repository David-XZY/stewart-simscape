function report=ch5Benchmark()
[c,root]=ch5Init();[m,s,ad,strict]=ch5Physics(c);
r=struct('c',c,'physical',m,'scene',s,'ad',ad,'strict',strict,'collision',ch5CollisionModel(m,s,c),'mode',"ideal-force");
r.plant=ch5PlantModel(m,s,ch5ActuatorProfile(r.mode,c,m),struct('Ts',c.Ts,'substeps',8,'Np',c.H,'integralGridTs',0));
q=m.qHome+[0;0;.15;zeros(3,1)];[d,G]=ad.dynamics(q,zeros(6,1));u=full(G)\(-full(d));x=[q;zeros(6,1)];
o=zeros(22,8);o(7:15,:)=repmat(reshape(eye(3),9,1),1,8);
build=tic;opt=ch5PredictiveOptimizer(r,[],true);report.build_seconds=toc(build);fprintf('Optimizer native build %.3fs\n',report.build_seconds);
z=[zeros(360,1);repmat(u,60,1);repmat(x,60,1);q;u];p=[x;u;q+[.05;0;0;zeros(3,1)];o(:);zeros(360,1)];
report.solve_seconds=zeros(3,1);report.violations=zeros(3,1);
for j=1:3
    solve=tic;sol=ch5SolveProjection(opt,z,p);report.solve_seconds(j)=toc(solve);z=full(sol.x);
    g=full(opt.check(z,p));report.violations(j)=max([opt.lower-g;g-opt.upper;0]);
    command=u+cumsum(reshape(z(1:360),6,60),2);nodes=full(r.plant.rollout(x,command));
    actual=z;actual(361:720)=command(:);actual(721:1440)=reshape(nodes(:,2:end),[],1);
    gg=full(opt.check(actual,p));eq=opt.lower==opt.upper;
    report.exact_scaled_equality(j)=max([abs(gg(eq)-opt.lower(eq))./opt.g_scale(eq);0]);
    report.exact_inequality(j)=max([opt.lower(~eq)-gg(~eq);gg(~eq)-opt.upper(~eq);0]);
    report.exact_terminal_velocity(j)=norm(nodes(7:12,end),inf);
    fprintf('Optimizer native solve %.3fs, violation %.3g\n',report.solve_seconds(j),report.violations(j));
end
report.stats=opt.solver.stats();save(fullfile(root,'chapter5','artifacts','validation','optimizer_benchmark.mat'),'report');
end
