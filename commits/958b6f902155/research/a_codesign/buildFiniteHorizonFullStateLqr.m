function schedule=buildFiniteHorizonFullStateLqr(A,B,Q,R,Qf)
% Discrete finite-horizon Riccati, supporting time-varying state/input costs.
n=size(A,1); N=size(A,3); m=size(B,2);
P=zeros(n,n,N+1); K=zeros(m,n,N); P(:,:,end)=Qf;
for k=N:-1:1
    q=Q(:,:,min(k,size(Q,3))); r=R(:,:,min(k,size(R,3)));
    S=r+B(:,:,k).'*P(:,:,k+1)*B(:,:,k); S=(S+S.')/2;
    [~,flag]=chol(S); assert(flag==0,'ABFD:Riccati','Riccati input Hessian is not positive definite.');
    K(:,:,k)=S\(B(:,:,k).'*P(:,:,k+1)*A(:,:,k));
    P(:,:,k)=q+A(:,:,k).'*P(:,:,k+1)*A(:,:,k)- ...
        A(:,:,k).'*P(:,:,k+1)*B(:,:,k)*K(:,:,k);
    P(:,:,k)=(P(:,:,k)+P(:,:,k).')/2;
end
schedule=struct('K',K,'P',P);
end
