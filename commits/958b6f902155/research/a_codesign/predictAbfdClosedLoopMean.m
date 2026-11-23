function prediction=predictAbfdClosedLoopMean(runtime,plantProvider,profile,options)
% Control-side mean prediction, not a new simulator or acceptance authority.
% Uses the actual research controller and known actuator parameters. Branch
% masses/compliance/friction enter only through the selected rigid/B provider.
if nargin<4, options=struct(); end
defaults=struct('maxStep',.001,'mode','closed-loop','queryTime',runtime.bundle.refs.t, ...
    'initialState',runtime.initialState,'initialForce',[], ...
    'commandTime',[],'commandForce',[],'temporalFile','','allowPilotTemporal',false);
for f=fieldnames(options).'
    assert(isfield(defaults,f{1}),'ABFD:PredictionOption','Unknown prediction option.');
    defaults.(f{1})=options.(f{1});
end
options=defaults; c=runtime.config; refs=runtime.bundle.refs;
assert(any(strcmp(c.controllerMode,{'fullstate-tvlqr','geometric-D'})) && ...
    ~c.qpEnabled && ~c.dobEnabled && ~c.antiWindupEnabled && ~c.governorEnabled && ...
    strcmp(c.stateSource,'direct'),'ABFD:MeanPredictionController', ...
    'This predictor requires direct-state TVLQR/D with optional loops disabled.');
options.mode=validatestring(options.mode,{'closed-loop','recorded-command'});
validateattributes(options.maxStep,{'double'},{'scalar','positive','finite'});
query=options.queryTime(:).'; T=query(end); dt=c.sampleTime;
assert(query(1)==0 && all(diff(query)>0) && T<=refs.t(end)+1e-10, ...
    'ABFD:MeanPredictionTime','Increasing query times must start at zero and stay in the reference.');
if isempty(options.initialForce)
    nominal=buildDynamicsProvider(runtime.bundle.model);
    options.initialForce=nominal.inverse(refs.q(:,1),refs.qd(:,1),refs.qdd(:,1));
    if isfield(profile,'initialAppliedForce'), options.initialForce=profile.initialAppliedForce; end
end
x=options.initialState(:); initialForce=options.initialForce(:);
validateattributes(x,{'double'},{'size',[12,1],'real','finite'});
validateattributes(initialForce,{'double'},{'size',[6,1],'real','finite'});
commandTime=0:dt:T; commands=nan(6,numel(commandTime));
if strcmp(options.mode,'recorded-command')
    commandTime=options.commandTime(:).'; commands=options.commandForce;
    assert(commandTime(1)==0 && all(diff(commandTime)>0) && ...
        isequal(size(commands),[6,numel(commandTime)]) && all(isfinite(commands(:))), ...
        'ABFD:PredictionCommand','Invalid recorded-command clock or values.');
end
events=unique([query,commandTime,commandTime+profile.forceDelay]); events=events(events<=T);
ad=buildCasadiCompositeDynamics(plantProvider);
temporal=[]; P=[]; stateCovariance=zeros(12,12,numel(query));
if ~isempty(options.temporalFile)
    assert(strcmp(plantProvider.mode,'residual-B') && profile.forceDelay<c.sampleTime, ...
        'ABFD:TemporalPredictionMode','Colored prediction requires B and actuator delay below one control period.');
    temporal=importResidualTemporalModel(options.temporalFile,c.residualFile,options.allowPilotTemporal,plantProvider);
    validateAbfdTemporalConditioning(plantProvider,temporal,profile);
    if isfield(temporal,'conditioning')
        assert(strcmp(runtime.provider.hash,plantProvider.hash) && ...
            norm(x(1:6)-plantProvider.biasCalibration.pose,inf)<1e-10, ...
            'ABFD:TemporalConditioningRuntime','Controller, predicted mean and initial observation differ.');
    end
    imported=importResidualNetworkWeights(c.residualFile);
    assert(strcmp(abfdHash(imported),abfdHash(plantProvider.network)), ...
        'ABFD:TemporalProviderMismatch','Prediction provider differs from the calibrated static network.');
    assert(abs(temporal.sampleTime-dt)<1e-8 && strcmp(options.mode,'closed-loop'), ...
        'ABFD:TemporalPredictionClock','Use the original residual clock and closed-loop mode.');
    initialTemporal=initializeAbfdTemporalState(temporal); temporalMean=initialTemporal.centeredMean;
    P=blkdiag(c.initialCovariance,zeros(24),initialTemporal.covariance);
    adStep=buildAbfdPredictionStepAD(plantProvider,profile,temporal.standardizedMean);
end
actuator=struct('slew',initialForce,'lag',initialForce); controllerState=struct();
states=zeros(12,numel(query)); actualForce=zeros(6,numel(query));
queryCommand=zeros(6,numel(query)); maximumRamp=0; integrations=0; timer=tic;
for k=1:numel(events)
    time=events(k);
    controlIndex=find(abs(commandTime-time)<1e-12,1,'last');
    if strcmp(options.mode,'closed-loop') && ~isempty(controlIndex)
        [commands(:,controlIndex),controllerState]=stepAbfdController(time,x,controllerState,runtime);
        if ~isempty(temporal)
            reset=eye(size(P)); reset(25:30,:)=0;
            gainIndex=find(runtime.design.schedule.t<=time+1e-12,1,'last');
            K=runtime.design.schedule.K(:,:,min(gainIndex,size(runtime.design.schedule.K,3)));
            C=-K;
            if strcmp(runtime.design.kind,'geometric')
                ref=[interp1(refs.t,refs.q.',time).';interp1(refs.t,refs.qd.',time).';interp1(refs.t,refs.qdd.',time).'];
                ref(4:6)=ref(4:6)+c.attitudePerturbation;
                C=full(runtime.design.policy.jacobian(x,ref,K));
            end
            reset(25:30,1:12)=C;
            if time>1e-12
                reset(37:end,37:end)=temporal.transition; temporalMean=temporal.transition*temporalMean;
            end
            P=reset*P*reset.';
            if time>1e-12, P(37:end,37:end)=P(37:end,37:end)+temporal.processCovariance; end
        end
    end
    if ~isempty(temporal) && any(abs(commandTime+profile.forceDelay-time)<1e-12)
        reset=eye(size(P)); reset(31:36,:)=0; reset(31:36,25:30)=eye(6); P=reset*P*reset.';
    end
    active=find(commandTime<=time+1e-12,1,'last');
    delayed=find(commandTime<=time-profile.forceDelay+1e-12,1,'last');
    input=initialForce; if ~isempty(delayed), input=commands(:,delayed); end
    assert(all(isfinite(input)),'ABFD:PredictionClock','Actuator requested an uncomputed command.');
    target=sign(input).*max(abs(input)-profile.forceDeadzone,0);
    ids=find(abs(query-time)<1e-12);
    if ~isempty(ids)
        [~,output]=advanceAbfdActuator(actuator,target,0,profile,plantProvider.model);
        states(:,ids)=repmat(x,1,numel(ids)); actualForce(:,ids)=repmat(output,1,numel(ids));
        queryCommand(:,ids)=repmat(commands(:,active),1,numel(ids));
        if ~isempty(temporal), stateCovariance(:,:,ids)=repmat(P(1:12,1:12),1,1,numel(ids)); end
        [H,~,Jv]=ad.terms(x(1:6),x(7:12));
        assert(rcond(full(H))>plantProvider.model.num.rcondMin && ...
            rcond(full(Jv))>plantProvider.model.num.rcondMin, ...
            'ABFD:PredictionSingular','Prediction became dynamically/transmission singular.');
    end
    if k==numel(events), break; end
    interval=events(k+1)-time; count=max(1,ceil(interval/options.maxStep)); h=interval/count;
    for j=1:count
        [~,f0]=advanceAbfdActuator(actuator,target,0,profile,plantProvider.model);
        [~,fm]=advanceAbfdActuator(actuator,target,h/2,profile,plantProvider.model);
        [next,f1,ramp]=advanceAbfdActuator(actuator,target,h,profile,plantProvider.model);
        if isempty(temporal)
            k1=full(ad.f(x,f0)); k2=full(ad.f(x+h*k1/2,fm));
            k3=full(ad.f(x+h*k2/2,fm)); k4=full(ad.f(x+h*k3,f1));
            x=x+h*(k1+2*k2+2*k3+k4)/6; actuator=next;
        else
            z=[x;actuator.slew;actuator.lag];
            [A,B,G]=adStep.jacobian(z,input,temporalMean(1:6),h);
            Phi=eye(size(P)); Phi(1:24,:)=0;
            Phi(1:24,1:24)=full(A); Phi(1:24,31:36)=full(B); Phi(1:24,37:42)=full(G);
            P=Phi*P*Phi.'; P(1:12,1:12)=P(1:12,1:12)+h*c.processSpectralDensity; P=(P+P.')/2;
            z=full(adStep.advance(z,input,temporalMean(1:6),h));
            x=z(1:12); actuator=struct('slew',z(13:18),'lag',z(19:24));
        end
        assert(all(isfinite(x)) && abs(cos(x(5)))>1e-5, ...
            'ABFD:PredictionInvalid','Prediction became nonfinite or reached an attitude singularity.');
        maximumRamp=max(maximumRamp,max(ramp)); integrations=integrations+1;
    end
end
prediction=struct('t',query,'x',states,'actualForce',actualForce,'commandForce',queryCommand, ...
    'commandTime',commandTime,'commands',commands,'elapsedSeconds',toc(timer), ...
    'integrationSteps',integrations,'maxStep',options.maxStep,'maximumRampDuration',maximumRamp, ...
    'mode',options.mode,'providerHash',plantProvider.hash,'profileHash',abfdHash(profile), ...
    'scope','rigid/B mean prediction with known actuator dynamics; no process noise or Simscape acceptance');
if ~isempty(temporal)
    prediction.P=stateCovariance; prediction.temporalAssetHash=temporal.assetHash;
    prediction.finalJointCovariance=P;
    prediction.temporalInitialization=initialTemporal.conditioning;
    prediction.scope='pilot local-linear covariance about biased mean; exact actuator-stage AD; sampled VAR wrench; no Simscape acceptance';
end
end
