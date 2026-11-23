function [cost,constraints,diagnostics]=buildAbfdSampledTrackingExpressions(X,A,F,t,provider,design,c,scene)
% Internal covariance [state; held command; held wrench error] prevents an
% off-clock HS knot from inventing another control/noise update.
import casadi.*
ad=buildCasadiCompositeDynamics(provider); leg=buildAbfdLegAccelerationAD(ad);
P=diagcat(MX(c.initialCovariance),MX.zeros(12,12));
cost=MX(0); constraints=MX.zeros(0,1); tube=cell(1,numel(t));
heldForce=cell(size(tube)); legVariance=cell(size(tube));
shooting=isfield(c,'covarianceShootingActive') && c.covarianceShootingActive;
directMoments=shooting && isfield(c,'covarianceCoordinates') && strcmp(c.covarianceCoordinates,'symmetric');
cutTimes=[]; factors=[]; cutEquality={}; cutPredictions={}; factorScale=[]; lower=[];
if shooting
    period=c.covarianceShootingInterval; steps=round(period/c.sampleTime);
    assert(steps>=1 && abs(steps*c.sampleTime-period)<1e-10,'ABFD:CovarianceCutClock');
    cutTimes=(steps:steps:floor(t(end)/c.sampleTime))*c.sampleTime;
    cutTimes=cutTimes(cutTimes<t(end)-1e-12);
    assert(~isempty(cutTimes),'ABFD:NoCovarianceCuts');
    lower=find(tril(true(12))); factors=MX.sym('covariance_factor',numel(lower),numel(cutTimes));
    factorScale=diag(sqrt(max(diag(c.initialCovariance),1e-12)));
    cutEquality=cell(size(cutTimes)); cutPredictions=cell(size(cutTimes));
end
assert(abs(t(1))<1e-10 && all(diff(t)>0),'ABFD:PredictionClock','Prediction starts at zero with increasing times.');
for k=1:numel(t)
    q=X(1:6,k); v=X(7:12,k); [H,~,~,Jq,L,Sigma]=ad.terms(q,v);
    [fx,fu]=ad.linearize(X(:,k),F(:,k)); Gc=-[MX.zeros(6,6);solve(H,MX.eye(6))];
    C=controllerJacobian(t(k),X(:,k),A(:,k),design);
    if isTick(t(k),c.sampleTime), P=updateAtTick(P,C,Sigma,t(k)); end
    stateP=P(1:12,1:12); forceCov=P(13:18,13:18);
    tube{k}=stateP; heldForce{k}=forceCov;
    errorCost=trace(c.Q*stateP); forceCost=trace(c.RF*forceCov);
    balance=F(:,k)-sum(F(:,k))/6;
    if k==1
        weight=(t(2)-t(1))/2;
    elseif k==numel(t)
        weight=(t(end)-t(end-1))/2;
    else
        weight=(t(k+1)-t(k-1))/2;
    end
    cost=cost+weight*(c.trackingWeight*errorCost+c.feedbackWeight*forceCost+ ...
        c.balanceWeight*(balance.'*c.RF*balance)+c.sensitivityWeight*trace(C.'*c.RF*C)/1e6);
    lengthJ=[Jq,MX.zeros(6,6)]; sdL=sqrt(diag(lengthJ*stateP*lengthJ.')+1e-16)*c.tubeSigma;
    sdF=sqrt(diag(forceCov)+1e-16)*c.tubeSigma;
    [vel,velJ,~,accX,accF,accW]=leg(X(:,k),F(:,k));
    sdV=sqrt(diag(velJ*stateP*velJ.')+1e-16)*c.tubeSigma;
    acceleration=velJ*[v;A(:,k)]; accelerationJ=[accX,accF,accW];
    accelerationCov=accelerationJ*P*accelerationJ.'; legVariance{k}=diag(accelerationCov);
    sdA=sqrt(diag(accelerationCov)+1e-16)*c.tubeSigma;
    constraints=[constraints;L+sdL-provider.model.lmax;provider.model.lmin-L+sdL; ...
        vel+sdV-provider.model.actuator.ldotMax;-vel+sdV-provider.model.actuator.ldotMax; ...
        F(:,k)+sdF-provider.model.actuator.forceMax;provider.model.actuator.forceMin-F(:,k)+sdF; ...
        acceleration+sdA-provider.model.actuator.lddotMax;-acceleration+sdA-provider.model.actuator.lddotMax]; %#ok<AGROW>
    if ~isempty(scene.hood.obstacles)
        nominalQ=design.nominal.q(:,min(k,size(design.nominal.q,2)));
        if isfield(design.nominal,'t')
            nominalQ=interp1(design.nominal.t,design.nominal.q.',t(k)).';
        end
        % One query already evaluates every obstacle at this same pose.
        distance=evaluateCylinderBoxDistanceNumeric(nominalQ,scene);
    end
    for j=1:numel(scene.hood.obstacles)
        normal=distance.bestNormals(:,j);
        gapFun=buildAbfdGapFunction(scene,scene.hood.obstacles(j),normal);
        [gap,gradient]=gapFun(q); grad=[gradient,MX.zeros(1,6)];
        required=scene.collision.safeDistance;
        if j==1
            required=scene.collision.stage1ConstraintDistance;
            if t(k)>scene.phase.durationApproach+1e-10, required=0; end
        end
        constraints=[constraints;required-gap+c.tubeSigma*sqrt(grad*stateP*grad.'+1e-16)]; %#ok<AGROW>
    end
    if k<numel(t)
        current=t(k);
        while current<t(k+1)-1e-12
            nextTick=(floor(current/c.sampleTime+1e-9)+1)*c.sampleTime;
            finish=min(nextTick,t(k+1)); h=finish-current;
            [Ad,input]=discretizeAbfdZoh(fx,[fu,Gc],h);
            Phi=[Ad,input;MX.zeros(12,12),MX.eye(12)];
            P=Phi*P*Phi.'+diagcat(h*MX(c.processSpectralDensity),MX.zeros(12,12));
            P=(P+P.')/2; current=finish;
            if current<t(k+1)-1e-12 && isTick(current,c.sampleTime)
                Cnext=controllerJacobian(current,X(:,k),A(:,k),design);
                P=updateAtTick(P,Cnext,Sigma,current);
            end
        end
    end
end
diagnostics=struct('P',{tube},'heldForceCovariance',{heldForce},'legAccelerationVariance',{legVariance}, ...
    'predictionModel','sampled rigid-body, held command and independent per-tick wrench error; interval-frozen linearization; actuator lag and temporal correlation not included');
if shooting
    assert(all(~cellfun(@isempty,cutEquality)),'ABFD:MissingCovarianceCut');
    diagonal=find(ismember(lower,1:13:144));
    diagnostics.lifting=struct('variables',factors(:),'equalities',vertcat(cutEquality{:}), ...
        'predictions',{cutPredictions},'scale',factorScale,'lowerIndices',lower, ...
        'diagonalFactorIndices',diagonal,'cutTimes',cutTimes,'factorSize',numel(lower), ...
        'scope','Exact state covariance continuity at control ticks; full within-segment held-command/wrench propagation');
    if directMoments
        diagnostics.lifting.kind='sampled-symmetric-shooting-v1';
        diagnostics.lifting.scope='Full symmetric state covariance coordinates; exact continuity, original process noise, no covariance projection or inflation';
    end
end
    function next=updateAtTick(previous,C,Sigma,time)
        import casadi.*
        if shooting
            index=find(abs(cutTimes-time)<1e-10);
            if ~isempty(index)
                assert(isscalar(index) && isempty(cutEquality{index}),'ABFD:DuplicateCovarianceCut');
                cutPredictions{index}=previous(1:12,1:12);
                factor=MX.zeros(12,12); factor(lower)=factors(:,index);
                normalized=solve(MX(factorScale),previous(1:12,1:12))/factorScale;
                if directMoments
                    covariance=factor+factor.'-diag(diag(factor));
                else
                    covariance=factor*factor.';
                end
                difference=normalized-covariance; cutEquality{index}=difference(lower);
                previous=diagcat(MX(factorScale)*covariance*factorScale,MX.zeros(12,12));
            end
        end
        next=sampleUpdate(previous,C,Sigma);
    end
end
function yes=isTick(time,dt)
yes=abs(time/dt-round(time/dt))<1e-8;
end
function C=controllerJacobian(time,x,a,design)
idx=find(design.schedule.t<=time+1e-12,1,'last');
K=design.schedule.K(:,:,min(idx,size(design.schedule.K,3)));
if strcmp(design.kind,'geometric'), C=design.policy.jacobian(x,[x;a],K); else, C=-K; end
end
function P=sampleUpdate(P,C,Sigma)
import casadi.*
reset=[MX.eye(12),MX.zeros(12,12);C,MX.zeros(6,12);MX.zeros(6,24)];
P=reset*P*reset.'+diagcat(MX.zeros(18,18),Sigma);
end
