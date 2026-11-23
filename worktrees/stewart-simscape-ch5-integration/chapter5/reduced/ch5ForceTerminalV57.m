function t=ch5ForceTerminalV57(q,r,k,verifyRegion)
% Previous command is part of the state; all Riccati calculations use scaled units.
% Region tests are numerical development evidence, NOT an invariance certificate.
if nargin<4,verifyRegion=false;end
n=r.plant.n;[us,Jus]=k.equilibrium(q);us=full(us);Jus=full(Jus);
xs=[q;zeros(6,1);repmat(us,(n-12)/6,1)];
[next,A,B]=k.stage(xs,us);A=full(A);B=full(B);next=full(next);
sc=[.02*ones(3,1);.05*ones(3,1);.1*ones(3,1);.2*ones(3,1);100*ones(n-12+6,1)];
T=diag(sc);Du=20*eye(6);Aa=[A,B;zeros(6,n),eye(6)];Ba=[B;eye(6)];
Ah=T\(Aa*T);Bh=T\(Ba*Du);[K,P,poles]=dlqr(Ah,Bh,eye(n+6),.2*eye(6));
Jsteady=[eye(6);zeros(6);repmat(Jus,(n-12)/6+1,1)];
uf=[zeros(6,n),eye(6)]*T-Du*K;df=-Du*K;
[mv,mx,mu]=k.mech(xs,us);mv=full(mv);mx=full(mx);mu=full(mu);
lo=[r.physical.lmin;-r.physical.actuator.ldotMax;-ch5PredictionAccelerationLimit(r);r.strict.sigmaSafe^2];
hi=[r.physical.lmax;r.physical.actuator.ldotMax;ch5PredictionAccelerationLimit(r);inf];
hiIndex=isfinite(hi);mg=mx*T(1:n,:)+mu*uf;
C=[mg(hiIndex,:);-mg;uf;-uf;df;-df];
slack=[hi(hiIndex)-mv(hiIndex);mv-lo;r.strict.forceMax-us;us-r.strict.forceMin; ...
    repmat(r.strict.forceRateLimit*r.c.Ts,2,1)];
L=chol(P,'lower');metric=C/L';slope=vecnorm(metric,2,2);
ratios=slack./max(slope,1e-14);rho=.25*min(ratios)^2;
valid=all(slack>0)&&isfinite(rho)&&rho>0&&max(abs(poles))<1&&norm(next-xs,inf)<1e-7;
if ~valid,rho=0;end
t=struct('q',q,'u',us,'x',xs,'xi',[xs;us],'scale',sc,'P',P,'K',K,'Du',Du,'rho',rho, ...
    'steady_jacobian',Jsteady,'linear_pole_radius',max(abs(poles)),'steady_residual',norm(next-xs,inf), ...
    'certified',false,'numerically_valid',valid,'region_verification',struct('performed',false));
if verifyRegion&&valid
    stream=RandStream('mt19937ar','Seed',5704);directions=[eye(n+6),-eye(n+6),randn(stream,n+6,96)];
    directions=directions./vecnorm(directions);directions=L'\directions;
    worst=inf;violation=inf;
    for attempt=1:10
        worst=-inf;violation=0;
        for j=1:size(directions,2)
            e=sqrt(t.rho)*directions(:,j);xi=t.xi+sc.*e;du=-Du*K*e;u=xi(n+1:end)+du;
            xn=full(r.plant.fd(xi(1:n),u,zeros(6,1)));en=([xn;u]-t.xi)./sc;
            decrease=en'*P*en-e'*P*e+e'*e;
            worst=max(worst,decrease);v=full(r.plant.intervalValues(xi(1:n),u));
            lower=[lo;-inf(3,1);r.strict.forceMin;-r.plant.profile.slewRate];
            upper=[hi;inf(3,1);r.strict.forceMax;r.plant.profile.slewRate];
            violation=max(violation,max([lower-v;v-upper; ...
                repmat(abs(du)-r.strict.forceRateLimit*r.c.Ts,1,size(v,2));zeros(1,size(v,2))],[],'all'));
        end
        if worst<=1e-7&&violation<=1e-6,break;end
        t.rho=t.rho/2;
    end
    t.region_verification=struct('performed',true,'directions',size(directions,2),'attempts',attempt, ...
        'maximum_lyapunov_residual',worst,'maximum_constraint_violation',violation,'accepted',worst<=1e-7&&violation<=1e-6, ...
        'scope','sampled_local_model_no_obstacles_no_global_certificate');
    t.numerically_valid=t.numerically_valid&&t.region_verification.accepted;
end
end
