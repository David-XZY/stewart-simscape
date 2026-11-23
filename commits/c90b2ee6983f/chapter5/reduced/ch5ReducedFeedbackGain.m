function K=ch5ReducedFeedbackGain(x,r,k)
% Numerical feedback preconditioning, not an externally supplied reference.
% U remains the applied force; its exact nonlinear rollout is audited.
[~,G]=r.ad.dynamics(x(1:6),zeros(6,1));[~,eqJac]=k.equilibrium(x(1:6));
inverseG=full(G)\eye(6);K=zeros(6,r.plant.n);
K(:,1:6)=16*inverseG-full(eqJac);K(:,7:12)=8*inverseG;
end
