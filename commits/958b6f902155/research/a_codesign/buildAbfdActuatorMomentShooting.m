function lifting=buildAbfdActuatorMomentShooting(t,c,temporal)
% Exact pre-control-tick coordinates: Y=[state;slew;lag;delayed command].
% Pending and delayed commands coincide before each cut because delay<dt.
% The exogenous VAR covariance is known; retain every Y/VAR cross moment.
% Direct symmetric coordinates admit the physically singular actuator prior.
% No Cholesky inflation, covariance reset, rank truncation or PSD projection.
import casadi.*
steps=round(c.covarianceShootingInterval/c.sampleTime);
assert(steps>=1 && abs(steps*c.sampleTime-c.covarianceShootingInterval)<1e-10, ...
    'ABFD:CovarianceCutClock','Moment cuts must coincide with the controller clock.');
cutTimes=(steps:steps:floor(t(end)/c.sampleTime))*c.sampleTime;
cutTimes=cutTimes(cutTimes<t(end)-1e-12);
assert(~isempty(cutTimes),'ABFD:NoCovarianceCuts','The horizon has no interior moment cuts.');
ny=30; nr=size(temporal.transition,1); lower=find(tril(true(ny)));
meanScale=[ones(12,1);100*ones(18,1)];
stateScale=[sqrt(max(diag(c.initialCovariance),1e-12));ones(18,1)];
residualScale=sqrt(diag(temporal.initialCovariance));
assert(numel(residualScale)==nr && all(isfinite(residualScale) & residualScale>0), ...
    'ABFD:MomentResidualScale','Use the positive stationary VAR diagonal for coordinate scaling.');
factorSize=ny+numel(lower)+ny*nr;
variables=MX.sym('actuator_moments',factorSize,numel(cutTimes));
embedding=zeros(36+nr,ny+nr);
embedding(1:24,1:24)=eye(24);
embedding(25:30,25:30)=eye(6); embedding(31:36,25:30)=eye(6);
embedding(37:end,31:end)=eye(nr);
lifting=struct('kind','actuator-moment-shooting-v1','variables',variables(:), ...
    'cutTimes',cutTimes,'factorSize',factorSize,'diagonalFactorIndices',[], ...
    'lowerIndices',lower,'meanScale',meanScale,'stateScale',diag(stateScale), ...
    'residualScale',diag(residualScale),'embedding',embedding,'residualCount',nr, ...
    'predictions',{cell(size(cutTimes))},'equationBlocks',{cell(size(cutTimes))}, ...
    'knownResidualCovariance',{cell(size(cutTimes))}, ...
    'scope','Exact pre-control mean, symmetric Y covariance and full Y/VAR cross covariance; exogenous VAR moments retained');
end
