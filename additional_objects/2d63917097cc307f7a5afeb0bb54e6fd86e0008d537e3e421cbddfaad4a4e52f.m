function report=ch5CheckControlApi()
% Exercise mechanics, denoising, projection, exact preview and force output.
% These explicitly untrained probe weights are never model checkpoints or
% teacher/formal data, and do not validate trained policy performance.
[c,root]=ch5Init();[m,s,ad,strict]=ch5Physics(c);
w=load(fullfile(root,'chapter5','artifacts','validation','network_api_probe.mat'));
report=struct('version',c.version,'trained_policy_validation',false);
report.config=c;
report.code_hashes=struct();
for name=["config.json","ch5DiffusionControl.m","ch5PredictiveOptimizer.m","ch5SolveProjection.m", ...
        "ch5InspectCandidate.m","ch5ControlInitialGuess.m","ch5PredictionAccelerationLimit.m","ch5DerivativeMap.m","ch5NativeNodes.m","ch5NativePreview.m","ch5CheckRollout.m", ...
        "ch5NativeCollisionAudit.m","ch5Denoise.m","ch5ProjectionJitOptions.m","compile_projection.py"]
    report.code_hashes.(strrep(char(name),'.','_'))=fileHash(fullfile(root,'chapter5',name));
end
actual=ch5Denoise(double(w.example_action),double(w.example_context),double(w.example_t),w);
report.inference_error=max(abs(actual-double(w.example_expected)),[],'all');assert(report.inference_error<1e-5);
w.action_mean=zeros(360,1);w.action_std=ones(360,1)*.001;
w.context_mean=zeros(219,1);w.context_std=ones(219,1);
grid=(0:c.diffusion_steps)';alpha=cos((grid/c.diffusion_steps+.008)/1.008*pi/2).^2;alpha=alpha/alpha(1);
beta=min(1-alpha(2:end)./alpha(1:end-1),.999);w.alpha_bar=cumprod(1-beta);
o=zeros(22,c.max_obstacles);o(7:15,:)=repmat(reshape(eye(3),9,1),1,c.max_obstacles);
for mode=["ideal-force","nonideal-force"]
    r=struct('c',c,'physical',m,'scene',s,'ad',ad,'strict',strict,'collision',ch5CollisionModel(m,s,c),'mode',mode,'weights',w);
    r.plant=ch5PlantModel(m,s,ch5ActuatorProfile(mode,c,m),struct('Ts',c.Ts,'substeps',8,'Np',c.H,'integralGridTs',0));
    q=m.qHome+[0;0;.15;zeros(3,1)];goal=q+[.05;0;0;zeros(3,1)];[d,G]=ad.dynamics(q,zeros(6,1));u=full(G)\(-full(d));
    x=[q;zeros(6,1)];if mode=="nonideal-force",x=[x;repmat(u,3,1)];end
    optimizer=ch5PredictiveOptimizer(r,[]);previewNodes=ch5NativeNodes(r);U=repmat(u,1,c.H);nodes=full(previewNodes(x,U));
    if mode=="nonideal-force"
        assert(isequal(optimizer.interval_constraint_times,r.plant.traceTime),'ch5:MissingInteriorConstraints','Every nonideal internal integration point must enter optimization.');
    end
    nativePreview=ch5NativePreview(r);nativeError=zeros(3,1);
    for nativeCase=1:2
        testU=U;
        if nativeCase==2,testU=testU+5*sin((1:c.H)*pi/c.H);end
        [vmX,vmValues,vmTrace]=r.plant.rollout(x,testU);
        [nativeX,nativeValues,nativeTrace]=nativePreview(x,testU);
        errors=[max(abs(full(vmX)-full(nativeX)),[],'all'); ...
            max(abs(full(vmValues)-full(nativeValues)),[],'all');max(abs(full(vmTrace)-full(nativeTrace)),[],'all')];
        nativeError=max(nativeError,errors);
    end
    assert(all(nativeError<1e-8),'ch5:NativePredictionMismatch','Native and VM physical rollout differ.');
    report.(['native_primal_',strrep(char(mode),'-','_')])=nativeError;
    z=[zeros(6*c.H,1);U(:);reshape(nodes(optimizer.shooting_rows,2:end),[],1);nodes(1:6,end);u];
    parameter=[x;u;goal;o(:);zeros(6*c.H,1)];g0=full(optimizer.check(z,parameter));
    perturbed=z;perturbed(12*c.H+1)=perturbed(12*c.H+1)+.001;
    g1=full(optimizer.check(perturbed,parameter));
    assert(abs(g1(7)-g0(7)-.001)<=1e-10,'ch5:LiftedDynamicsMissing','The lifted state must remain constrained by fd.');
    setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',9821));
    if isappdata(0,'ch5DiffusionWarm'),rmappdata(0,'ch5DiffusionWarm');end
    records=cell(3,1);
    for k=1:3
        time=(k-1+(k==3))*c.Ts;
        if k==3,assert(isappdata(0,'ch5DiffusionWarm'),'ch5:NoWarmPlanForGapProbe','A plan is required to exercise the skipped sample.');end
        timer=tic;[command,diagnostic,prediction]=ch5DiffusionControl(x,u,goal,o,r,time);
        records{k}=struct('seconds',toc(timer),'diagnostic',diagnostic,'prediction',prediction,'command',command,'x',x);
        assert(all(isfinite(command))&&all(command>=strict.forceMin-1e-6&command<=strict.forceMax+1e-6));
        assert(all(abs(command-u)<=strict.forceRateLimit*c.Ts+1e-6));
        attempts=prediction.denoise_attempts;
        assert(all(cellfun(@(a)~strcmp(a.status,'exception'),attempts),'all'), ...
            'ch5:ProbeImplementationError','A denoising/projection exception cannot pass the API check.');
        if k==3
            assert(strcmp(prediction.initial_guess.source,'shifted_plan')&&prediction.initial_guess.shift_steps==2, ...
                'ch5:WarmPlanLostAfterGap','The preceding plan must survive one held-input sample.');
        end
        fprintf('Untrained API probe %s step %d: fallback=%d, %.3fs\n',mode,k,diagnostic.usedFallback,records{k}.seconds);
        x=full(r.plant.fd(x,command,zeros(6,1)));u=command;
        if k==2,x=full(r.plant.fd(x,command,zeros(6,1)));end
    end
    assert(norm(x(1:3)-goal(1:3))<norm(q(1:3)-goal(1:3))-1e-7, ...
        'ch5:ProbeNoProgress','The obstacle-free force-control probe must make measurable progress toward its goal.');
    label=strrep(char(mode),'-','_');report.(label)=records;
end
report.passed=true;save(fullfile(root,'chapter5','artifacts','validation','control_api.mat'),'report','-v7');
end
function value=fileHash(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),digest.update(fread(fid,1048576,'*uint8'));end
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
