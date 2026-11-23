function rollout=buildAbfdActuatorRolloutAD(X,A,t,provider,design,c,profile,temporal,maxStep,traceEnabled)
% Full candidate-dependent mean/covariance rollout for the optimization bridge.
% All dynamics, reference derivatives and the full D policy remain symbolic.
% Asset import/acceptance belongs to the caller; this low-level kernel does not
% infer or replace missing residual models.
import casadi.*
if nargin<10, traceEnabled=false; end
traceP={}; traceInput={}; traceTime=[]; traceStep=[];
validateAbfdTemporalConditioning(provider,temporal,profile);
assert(strcmp(provider.mode,'residual-B') && abs(temporal.sampleTime-c.sampleTime)<1e-8 && ...
    profile.forceDelay<c.sampleTime,'ABFD:RolloutModel','Use B, matching clocks and sub-period delay.');
assert(~c.qpEnabled && ~c.dobEnabled && ~c.antiWindupEnabled && ~c.governorEnabled && ...
    strcmp(c.stateSource,'direct'),'ABFD:RolloutControl','Optional loops/observers need their own full state model.');
validateattributes(maxStep,{'double'},{'scalar','positive','finite'});
t=t(:).'; assert(t(1)==0 && all(diff(t)>0),'ABFD:RolloutTime','Increasing rollout time must start at zero.');
clock=0:c.sampleTime:t(end); controlRef=evaluateAbfdHermiteExpressions(X,A,t,clock);
nominal=buildCasadiCompositeDynamics(buildDynamicsProvider(provider.nominalModel));
ad=buildCasadiCompositeDynamics(provider); step=buildAbfdPredictionStepAD(provider,profile,temporal.standardizedMean);
preload=nominal.inverse(X(1:6,1),X(7:12,1),A(:,1));
if isfield(profile,'initialAppliedForce'), preload=MX(profile.initialAppliedForce); end
z=[X(1:6,1)+profile.initialPoseOffset;MX.zeros(6,1);preload;preload];
pending=preload; delayed=preload; nominalCommand=preload;
initialTemporal=initializeAbfdTemporalState(temporal); temporalMean=initialTemporal.centeredMean;
P=diagcat(MX(c.initialCovariance),MX.zeros(24,24),MX(initialTemporal.covariance)); n=size(P,1);
useShooting=isfield(c,'covarianceShootingActive') && c.covarianceShootingActive;
residualP=initialTemporal.covariance;
if useShooting, lifting=buildAbfdActuatorMomentShooting(t,c,temporal); end
% After a cut, P(t)=M(t)*Pcut*M(t)'+Q(t). M and Q depend on the
% candidate mean/reference, but never on the free covariance coordinates.
% Keep those coordinates outside the repeated step derivative graphs.
affine=false; momentBase=[]; momentMap=[]; momentNoise=[];
points=cell(size(t)); covariances=cell(size(t)); commands=cell(size(t)); nominalCommands=cell(size(t));
heldJacobians=cell(size(t)); C=MX.zeros(6,12);
temporalMeans=zeros(size(temporalMean,1),numel(t));
events=unique([t,clock,clock+profile.forceDelay]); events=events(events<=t(end));
for k=1:numel(events)
    time=events(k); index=find(abs(clock-time)<1e-12,1,'last');
    if useShooting && ~isempty(index)
        cut=find(abs(lifting.cutTimes-time)<1e-12,1);
        if ~isempty(cut)
            if affine, P=materialize(momentMap,momentBase,momentNoise); end
            [z,pending,delayed,P,lifting,momentBase]=applyAbfdActuatorMomentCut(z,delayed,P,residualP,lifting,cut);
            momentMap=MX(lifting.embedding); momentNoise=MX.zeros(n,n); affine=true;
        end
    end
    if ~isempty(index)
        ref=controlRef(:,index); ref(4:6)=ref(4:6)+c.attitudePerturbation;
        gainIndex=find(design.schedule.t<=time+1e-12,1,'last');
        K=design.schedule.K(:,:,min(gainIndex,size(design.schedule.K,3)));
        nominalCommand=ad.inverse(ref(1:6),ref(7:12),ref(13:18));
        if strcmp(design.kind,'geometric')
            pending=design.policy.force(z(1:12),ref,K); C=design.policy.jacobian(z(1:12),ref,K);
        else
            pending=nominalCommand-K*(z(1:12)-ref(1:12)); C=-K;
        end
        reset=MX.eye(n); reset(25:30,:)=MX.zeros(6,n); reset(25:30,1:12)=C;
        if time>1e-12
            reset(37:end,37:end)=temporal.transition; temporalMean=temporal.transition*temporalMean;
            residualP=temporal.transition*residualP*temporal.transition.'+temporal.processCovariance;
            residualP=(residualP+residualP.')/2;
        end
        if affine
            momentMap=reset*momentMap; momentNoise=reset*momentNoise*reset.';
            if time>1e-12, momentNoise(37:end,37:end)=momentNoise(37:end,37:end)+temporal.processCovariance; end
        else
            P=reset*P*reset.';
            if time>1e-12, P(37:end,37:end)=P(37:end,37:end)+temporal.processCovariance; end
        end
    end
    if any(abs(clock+profile.forceDelay-time)<1e-12)
        delayed=pending; reset=MX.eye(n); reset(31:36,:)=MX.zeros(6,n); reset(31:36,25:30)=MX.eye(6);
        if affine
            momentMap=reset*momentMap; momentNoise=reset*momentNoise*reset.';
        else
            P=reset*P*reset.';
        end
    end
    ids=find(abs(t-time)<1e-12);
    if affine && ~isempty(ids), P=materialize(momentMap,momentBase,momentNoise); end
    for j=ids
        points{j}=z; covariances{j}=P; commands{j}=pending; nominalCommands{j}=nominalCommand;
        heldJacobians{j}=C;
        temporalMeans(:,j)=temporalMean;
    end
    if k==numel(events), break; end
    count=max(1,ceil((events(k+1)-time)/maxStep)); h=(events(k+1)-time)/count;
    for j=1:count
        [Az,Bz,Gz]=step.jacobian(z,delayed,temporalMean(1:6),h);
        Phi=MX.eye(n); Phi(1:24,:)=MX.zeros(24,n);
        Phi(1:24,1:24)=Az; Phi(1:24,31:36)=Bz; Phi(1:24,37:42)=Gz;
        if affine
            momentMap=Phi*momentMap; momentNoise=Phi*momentNoise*Phi.';
            momentNoise(1:12,1:12)=momentNoise(1:12,1:12)+h*c.processSpectralDensity;
            momentNoise=(momentNoise+momentNoise.')/2;
        else
            P=Phi*P*Phi.'; P(1:12,1:12)=P(1:12,1:12)+h*c.processSpectralDensity;
            P=(P+P.')/2;
        end
        z=step.advance(z,delayed,temporalMean(1:6),h);
        if traceEnabled
            if affine, P=materialize(momentMap,momentBase,momentNoise); end
            traceP{end+1}=P; traceInput{end+1}=[z;delayed]; %#ok<AGROW>
            traceTime(end+1)=time+j*h; traceStep(end+1)=h; %#ok<AGROW>
        end
    end
end
rollout=struct('mean',[points{:}],'P',{covariances},'command',[commands{:}], ...
    'temporalCenteredMean',temporalMeans, ...
    'nominalCommand',[nominalCommands{:}],'commandJacobian',{heldJacobians},'reference',[X;A],'t',t, ...
    'scope','full candidate-dependent actuator and VAR propagation with frozen gain schedule; low-level optimization kernel');
if useShooting
    assert(all(~cellfun(@isempty,lifting.equationBlocks)), ...
        'ABFD:MissingMomentCut','Every configured moment cut must be visited exactly once.');
    lifting.equalities=vertcat(lifting.equationBlocks{:});
    lifting.propagation='transition-affine-v1';
    rollout.lifting=rmfield(lifting,'equationBlocks');
end
if traceEnabled
    rollout.trace=struct('P',{traceP},'input',{traceInput},'time',traceTime,'step',traceStep);
end
end
function P=materialize(map,base,noise)
P=map*base*map.'+noise; P=(P+P.')/2;
end
