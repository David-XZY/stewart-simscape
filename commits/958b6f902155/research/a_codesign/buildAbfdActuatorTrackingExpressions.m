function [cost,constraints,diagnostics]=buildAbfdActuatorTrackingExpressions(X,A,t,provider,design,c,scene)
% Full nonlinear candidate-dependent actuator/VAR mean and covariance, with
% physical force and raw feedback-command budgets checked separately.
import casadi.*
allow=false; if isfield(c,'allowPilotTemporal'), allow=c.allowPilotTemporal; end
temporal=importResidualTemporalModel(c.temporalFile,c.residualFile,allow,provider);
imported=importResidualNetworkWeights(c.residualFile);
assert(strcmp(abfdHash(imported),abfdHash(provider.network)), ...
    'ABFD:TemporalProviderMismatch','Optimization provider differs from the calibrated network.');
profile=makeHighFidelityPlantProfile(c.plantProfile,c.plantOverrides);
maxStep=c.sampleTime/10; if isfield(c,'predictionMaxStep'), maxStep=c.predictionMaxStep; end
rollout=buildAbfdActuatorRolloutAD(X,A,t,provider,design,c,profile,temporal,maxStep);
kin=buildAbfdUncertainKinematicsAD(provider,temporal.standardizedMean);
count=numel(t); cost=MX(0); constraints=cell(1,count); stateP=cell(1,count);
rootArguments=cell(1,count); affineConstraints=cell(1,count);
obstacleCount=numel(scene.hood.obstacles);
rootColumns=[1:6,1:6,7:12,7:12,13:18,13:18,19:24,19:24, ...
    25:30,25:30,25:30,25:30,31:30+obstacleCount];
rootRows=[1:72,74:73+obstacleCount];
rootMap=sparse(rootRows,rootColumns,c.tubeSigma,73+obstacleCount,30+obstacleCount);
gapFunctions=cell(numel(scene.hood.obstacles),2);
for j=1:numel(scene.hood.obstacles)
    gapFunctions{j,1}=buildAbfdPredictionGapAD(scene,scene.hood.obstacles(j),j,false);
    gapFunctions{j,2}=buildAbfdPredictionGapAD(scene,scene.hood.obstacles(j),j,true);
end
balanceMap=eye(6)-ones(6)/6;
for k=1:count
    z=rollout.mean(:,k); P=rollout.P{k}; stateP{k}=P(1:12,1:12);
    [values,Jz,Jr]=kin(z,rollout.temporalCenteredMean(1:6,k));
    J=[Jz,MX.zeros(24,12),Jr,MX.zeros(24,size(P,1)-42)];
    variance=J*P*J.';
    commandP=P(25:30,25:30);
    feedback=rollout.command(:,k)-rollout.nominalCommand(:,k);
    limits=provider.model.actuator;
    g=[values(1:6)-provider.model.lmax;provider.model.lmin-values(1:6); ...
        values(7:12)-limits.ldotMax;-values(7:12)-limits.ldotMax; ...
        values(13:18)-limits.lddotMax;-values(13:18)-limits.lddotMax; ...
        values(19:24)-limits.forceMax;limits.forceMin-values(19:24); ...
        rollout.command(:,k)-limits.forceMax;limits.forceMin-rollout.command(:,k); ...
        feedback-c.feedbackBudget;-feedback-c.feedbackBudget; ...
        1e-10-cos(z(5))^2];
    gapConstraints=cell(obstacleCount,1); gapArguments=cell(obstacleCount,1);
    if ~isempty(scene.hood.obstacles)
        nominal=interp1(design.nominal.t,design.nominal.q.',t(k)).';
        distance=evaluateCylinderBoxDistanceNumeric(nominal,scene);
        phase=1+double(t(k)>scene.phase.durationApproach+1e-10);
        for j=1:numel(scene.hood.obstacles)
            [clearance,gradient]=gapFunctions{j,phase}(z(1:6),distance.bestNormals(:,j));
            stateGradient=[gradient,MX.zeros(1,6)];
            gapConstraints{j}=-clearance;
            gapArguments{j}=stateGradient*stateP{k}*stateGradient.'+1e-16;
        end
    end
    rootArguments{k}=[diag(variance)+1e-16;diag(commandP)+1e-16;vertcat(gapArguments{:})];
    affineConstraints{k}=[g;vertcat(gapConstraints{:})];
    constraints{k}=affineConstraints{k}+rootMap*sqrt(rootArguments{k});
    error=z(1:12)-X(:,k); balance=balanceMap*values(19:24);
    if k==1
        weight=(t(2)-t(1))/2;
    elseif k==count
        weight=(t(end)-t(end-1))/2;
    else
        weight=(t(k+1)-t(k-1))/2;
    end
    C=rollout.commandJacobian{k};
    cost=cost+weight*(c.trackingWeight*(error.'*c.Q*error+trace(c.Q*stateP{k}))+ ...
        c.feedbackWeight*(feedback.'*c.RF*feedback+trace(c.RF*commandP))+ ...
        c.balanceWeight*(balance.'*c.RF*balance+trace(c.RF*balanceMap*variance(19:24,19:24)*balanceMap.'))+ ...
        c.sensitivityWeight*trace(C.'*c.RF*C)/1e6);
end
constraints=vertcat(constraints{:});
diagnostics=struct('P',{stateP},'mean',rollout.mean(1:12,:), ...
    'temporalAssetHash',temporal.assetHash,'predictionMaxStep',maxStep, ...
    'predictionModel','candidate-dependent nonlinear mean; exact actuator stage derivatives; sampled VAR; local linear covariance; empirical acceptance required');
if isfield(rollout,'lifting'), diagnostics.lifting=rollout.lifting; end
diagnostics.rootCurvature=struct('arguments',vertcat(rootArguments{:}), ...
    'affineConstraints',vertcat(affineConstraints{:}),'constraintMap',kron(speye(count),rootMap));
end
