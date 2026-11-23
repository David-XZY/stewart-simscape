function [corrected,Pnext,predicted]=stepAbfdIntervalObserver(posterior,P,pose,forceImpulse,provider,dt)
% Predict across the elapsed interval using measured leg-force impulse (N s),
% then correct at its end. No future force is inferred from a past sample.
% RK4 uses the interval-average force; covariance differentiates that same map.
import casadi.*
persistent cachedHash cachedStep cachedTerms
validateattributes(posterior,{'double'},{'size',[12,1],'real','finite'});
validateattributes(P,{'double'},{'size',[12,12],'real','finite'});
validateattributes(pose,{'double'},{'size',[6,1],'real','finite'});
validateattributes(forceImpulse,{'double'},{'size',[6,1],'real','finite'});
validateattributes(dt,{'double'},{'scalar','real','finite','nonnegative'});
assert(norm(P-P.','fro')<1e-10 && min(eig((P+P.')/2))>=-1e-12, ...
    'ABFD:ObserverCovariance','Invalid observer posterior covariance.');
predicted=posterior; Ppred=P;
if dt==0
    assert(norm(forceImpulse,inf)<1e-12,'ABFD:ObserverImpulseClock','A zero interval cannot contain force impulse.');
else
    if isempty(cachedHash) || ~strcmp(cachedHash,provider.hash)
        ad=buildCasadiCompositeDynamics(provider); x=SX.sym('x',12); force=SX.sym('F',6); h=SX.sym('h');
        k1=ad.f(x,force); x2=x+h/2*k1; k2=ad.f(x2,force);
        x3=x+h/2*k2; k3=ad.f(x3,force); x4=x+h*k3; k4=ad.f(x4,force);
        next=x+h/6*(k1+2*k2+2*k3+k4);
        cachedStep=Function('abfd_interval_observer',{x,force,h},{next,jacobian(next,x),[x,x2,x3,x4]});
        cachedTerms=ad.terms; cachedHash=provider.hash;
    end
    [value,transition,stages]=cachedStep(posterior,forceImpulse/dt,dt);
    predicted=full(value); transition=full(transition); stages=full(stages);
    assert(all(isfinite([predicted;transition(:);stages(:)])),'ABFD:ObserverNonfinite','Nonfinite observer prediction or derivative.');
    for k=1:size(stages,2)
        E=rpyRateMapZYX(stages(4:6,k)); [H,~,Jv]=cachedTerms(stages(1:6,k),stages(7:12,k));
        assert(rcond(E)>1e-5 && rcond(full(H))>provider.model.num.rcondMin && ...
            rcond(full(Jv))>provider.model.num.rcondMin, ...
            'ABFD:ObserverSingular','Ill-conditioned observer integration stage.');
    end
    Ppred=transition*P*transition.'+dt*1e-6*eye(12); Ppred=(Ppred+Ppred.')/2;
end
% Reuse only the pose correction (dt=0), with the original sensor covariance
% and world-frame SO(3) innovation. It performs no additional prediction.
[corrected,~,Pnext]=stepAbfdExternalObserver(predicted,Ppred,pose,zeros(6,1),provider,0);
end
