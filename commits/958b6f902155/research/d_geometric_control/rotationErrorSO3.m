function e=rotationErrorSO3(R,Rref)
% World tangent Log(R*Rref'). At pi the axis sign is intrinsically ambiguous.
S=R*Rref.';
assert(norm(S.'*S-eye(3),'fro')<1e-6 && det(S)>0,'ABFD:Rotation','Invalid rotation.');
c=max(-1,min(1,(trace(S)-1)/2)); theta=acos(c);
vee=[S(3,2)-S(2,3);S(1,3)-S(3,1);S(2,1)-S(1,2)];
if theta<1e-6
    e=(0.5+theta^2/12)*vee;
elseif pi-theta<1e-5
    [V,D]=eig((S+eye(3))/2); [~,i]=max(real(diag(D))); axis=real(V(:,i));
    [~,j]=max(abs(axis)); if axis(j)<0, axis=-axis; end
    if norm(vee)>1e-10 && dot(axis,vee)<0, axis=-axis; end
    e=theta*axis/norm(axis);
else
    e=theta/(2*sin(theta))*vee;
end
end
