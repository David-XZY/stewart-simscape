function lin=linearizeTrajectoryAD(refs,provider)
ad=buildCasadiCompositeDynamics(provider); N=numel(refs.t)-1;
A=zeros(12,12,N); B=zeros(12,6,N); Ac=A; Bc=B;
for k=1:N
    [aa,bb]=ad.linearize([refs.q(:,k);refs.qd(:,k)],refs.Fleg(:,k));
    Ac(:,:,k)=full(aa); Bc(:,:,k)=full(bb);
    dt=refs.t(k+1)-refs.t(k); Z=expm([full(aa),full(bb);zeros(6,18)]*dt);
    A(:,:,k)=Z(1:12,1:12); B(:,:,k)=Z(1:12,13:18);
end
lin=struct('A',A,'B',B,'Ac',Ac,'Bc',Bc,'t',refs.t);
end
