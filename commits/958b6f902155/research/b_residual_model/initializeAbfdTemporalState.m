function state=initializeAbfdTemporalState(temporal)
% Keep stationary VAR parameters; condition its history on the observed e(0)=0.
% A noiseless static-balance observation is available only for the explicit
% calibration-assisted simulation protocol. No trajectory data is used here.
P=temporal.initialCovariance; n=size(P,1);
state=struct('centeredMean',zeros(n,1),'covariance',P,'conditioning','stationary prior');
if ~isfield(temporal,'conditioning'), return; end
assert(any(strcmp(temporal.conditioning.kind,{'independent-initial-static-balance-v1', ...
    'independent-initial-inertia-static-v2'})), ...
    'ABFD:TemporalInitialObservation','Unsupported initial residual observation.');
observed=P(1:6,1:6);
assert(rcond(observed)>1e-12 && min(eig(observed))>0,'ABFD:TemporalInitialObservation', ...
    'The observed stationary covariance must be nonsingular.');
% Block Cholesky gives the Schur complement without subtracting two nearly
% equal stationary covariances. Its observed block is analytically zero.
L=chol(P,'lower'); gain=[eye(6);L(7:end,1:6)/L(1:6,1:6)];
state.centeredMean=-gain*temporal.standardizedMean(:);
state.covariance=blkdiag(zeros(6),L(7:end,7:end)*L(7:end,7:end).');
assert(norm(state.centeredMean(1:6)+temporal.standardizedMean(:))<1e-9 && ...
    norm(state.covariance(1:6,:),'fro')<1e-8*max(1,norm(P,'fro')) && ...
    min(eig(state.covariance))>=-1e-9*max(1,norm(P,2)), ...
    'ABFD:TemporalInitialObservation','Gaussian conditioning failed its mean/covariance identity.');
state.conditioning='Gaussian history conditioned on exact initial zero wrench residual; observation noise assumed zero; state/residual prior independent';
end
