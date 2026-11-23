probeDirectory=fileparts(mfilename('fullpath'));
addpath(fileparts(fileparts(probeDirectory)));addpath(probeDirectory);
[c,root]=ch5Init();[m,s,ad,strict]=ch5Physics(c);mode="nonideal-force";
r=struct('c',c,'physical',m,'scene',s,'ad',ad,'strict',strict,'mode',mode,'collision',ch5CollisionModel(m,s,c));
r.plant=ch5PlantModel(m,s,ch5ActuatorProfile(mode,c,m),struct('Ts',c.Ts,'substeps',8,'Np',c.H,'integralGridTs',0));
q=m.qHome+[0;0;.15;zeros(3,1)];[d,G]=ad.dynamics(q,zeros(6,1));u=full(G)\(-full(d));
x=[q;zeros(6,1);repmat(u,3,1)];U=repmat(u,1,c.H);nodesFunction=ch5NativeNodes(r);nodes=full(nodesFunction(x,U));
o=zeros(22,c.max_obstacles);o(7:15,:)=repmat(reshape(eye(3),9,1),1,c.max_obstacles);
parameter=[x;u;q+[.05;0;zeros(4,1)];o(:);zeros(6*c.H,1)];reference=ch5PredictiveOptimizer(r,[]);
z=[zeros(6*c.H,1);U(:);reshape(nodes(reference.shooting_rows,2:end),[],1);q;u];y=z./reference.x_scale;
[g0,f0,df0,J0]=reference.linearization(y,parameter);report=struct();
for width=[4,1,2]
    if width==4,optimizer=reference;else,fn=str2func(sprintf('ch5ProjectionLane%d',width));optimizer=fn(r,[]);end
    [g,f,df,J]=optimizer.linearization(y,parameter);
    err=max([norm(full(g-g0),inf),abs(full(f-f0)),norm(full(df-df0),inf),norm(full(J-J0),inf)]);assert(err<1e-8);
    times=zeros(10,1);
    for k=1:10,timer=tic;optimizer.linearization(y,parameter);times(k)=toc(timer);end
    report.(sprintf('lane_%d',width))=struct('error',err,'seconds',times);
    fprintf('AD lane width %d: median %.6fs, error %.3g\n',width,median(times),err);
    save(fullfile(root,'chapter5','artifacts','validation','lane_benchmark.mat'),'report','-v7');
end
