function [U,info]=ch5ReducedTerminal(x,U,r,k)
% Preserve the complete shifted sequence; correct only its final two blocks.
% This Newton correction is a numerical operation, never an acceptance test.
H=r.c.H;first=H-9;info=struct('iterations',0,'residual',inf,'finite',true);
for iteration=1:3
    nodes=full(k.nodes(x,U));
    if ~all(isfinite(nodes),'all'),info.finite=false;return;end
    [ua,eqJac]=k.equilibrium(nodes(1:6,end));ua=full(ua);eqJac=full(eqJac);
    residual=[nodes(7:12,end);(U(:,end)-ua)/1000];info.residual=norm(residual,inf);
    if info.residual<1e-8,return;end
    S=zeros(r.plant.n,12);
    for j=first:H
        [~,A,B]=k.stage(nodes(:,j),U(:,j));A=full(A);B=full(B);
        S=A*S;cols=1:6;if j>H-5,cols=7:12;end;S(:,cols)=S(:,cols)+B;
    end
    F=[S(7:12,:);([zeros(6),eye(6)]-eqJac*S(1:6,:))/1000];
    if rcond(F)<1e-12,info.finite=false;return;end
    correction=-F\residual;
    if ~all(isfinite(correction))||norm(correction,inf)>200,info.finite=false;return;end
    U(:,first:H-5)=U(:,first:H-5)+correction(1:6);
    U(:,H-4:H)=U(:,H-4:H)+correction(7:12);info.iterations=iteration;
end
end
