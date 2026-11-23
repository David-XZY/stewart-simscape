function [cost,constraints,diagnostics]=buildAbfdTrackingExpressions(X,A,F,t,provider,design,c,scene)
% Frozen gains; candidate-dependent dynamics AND full D state Jacobian.
if isfield(c,'predictionProfile') && strcmp(c.predictionProfile,'actuator-var-v3')
    [cost,constraints,diagnostics]=buildAbfdActuatorTrackingExpressions(X,A,t,provider,design,c,scene);
    return;
end
assert(~isfield(provider,'biasCalibration'),'ABFD:UncalibratedConditionalUncertainty', ...
    'Static mean conditioning requires its matched actuator/VAR uncertainty model.');
if isfield(c,'predictionProfile') && strcmp(c.predictionProfile,'sampled-hold-v2')
    [cost,constraints,diagnostics]=buildAbfdSampledTrackingExpressions(X,A,F,t,provider,design,c,scene);
    return;
end
import casadi.*
ad=buildCasadiCompositeDynamics(provider); P=MX(c.initialCovariance);
cost=MX(0); constraints=MX.zeros(0,1); tube=cell(1,numel(t));
for k=1:numel(t)
    tube{k}=P; q=X(1:6,k); v=X(7:12,k);
    idx=find(design.schedule.t<=t(k)+1e-12,1,'last');
    K=design.schedule.K(:,:,min(idx,size(design.schedule.K,3)));
    [H,~,~,Jq,L,Sigma]=ad.terms(q,v);
    if strcmp(design.kind,'geometric')
        C=design.policy.jacobian(X(:,k),[X(:,k);A(:,k)],K);
    else
        C=-K;
    end
    errorCost=trace(c.Q*P); forceCov=C*P*C.';
    forceCost=trace(c.RF*forceCov);
    balance=F(:,k)-sum(F(:,k))/6;
    if k==1
        dt=(t(2)-t(1))/2;
    elseif k==numel(t)
        dt=(t(end)-t(end-1))/2;
    else
        dt=(t(k+1)-t(k-1))/2;
    end
    cost=cost+dt*(c.trackingWeight*errorCost+c.feedbackWeight*forceCost+ ...
        c.balanceWeight*(balance.'*c.RF*balance)+c.sensitivityWeight*trace(C.'*c.RF*C)/1e6);
    lengthJ=[Jq,MX.zeros(6,6)]; sdL=sqrt(diag(lengthJ*P*lengthJ.')+1e-16)*c.tubeSigma;
    sdF=sqrt(diag(forceCov)+1e-16)*c.tubeSigma;
    % Velocity Jacobian must be differentiated against an independent SX state.
    sx=SX.sym('tube_state',12); [~,~,~,jq]=ad.terms(sx(1:6),sx(7:12));
    vel=jq*sx(7:12); velFun=Function('tube_velocity',{sx},{vel,jacobian(vel,sx)});
    [vel,velJ]=velFun(X(:,k)); sdV=sqrt(diag(velJ*P*velJ.')+1e-16)*c.tubeSigma;
    constraints=[constraints; L+sdL-provider.model.lmax;provider.model.lmin-L+sdL; ...
        vel+sdV-provider.model.actuator.ldotMax;-vel+sdV-provider.model.actuator.ldotMax; ...
        F(:,k)+sdF-provider.model.actuator.forceMax;provider.model.actuator.forceMin-F(:,k)+sdF]; %#ok<AGROW>
    % Fixed support normals give conservative differentiable separating planes.
    for j=1:numel(scene.hood.obstacles)
        nominalQ=design.nominal.q(:,min(k,size(design.nominal.q,2)));
        distance=evaluateCylinderBoxDistanceNumeric(nominalQ,scene); normal=distance.bestNormals(:,j);
        gapFun=buildAbfdGapFunction(scene,scene.hood.obstacles(j),normal);
        [gap,gradient]=gapFun(q); grad=[gradient,MX.zeros(1,6)];
        required=scene.collision.safeDistance;
        if j==1
            required=scene.collision.stage1ConstraintDistance;
            if t(k)>scene.phase.durationApproach+1e-10, required=0; end
        end
        constraints=[constraints;required-gap+c.tubeSigma*sqrt(grad*P*grad.'+1e-16)]; %#ok<AGROW>
    end
    if k<numel(t)
        [fx,fu]=ad.linearize(X(:,k),F(:,k)); Ac=fx+fu*C;
        Gc=[MX.zeros(6,6);solve(H,MX.eye(6))];
        [Acl,G]=discretizeAbfdClosedLoop(Ac,Gc,t(k+1)-t(k));
        P=Acl*P*Acl.'+G*Sigma*G.'+(t(k+1)-t(k))*c.processSpectralDensity;
        P=(P+P.')/2;
    end
end
diagnostics=struct('P',{tube});
end
