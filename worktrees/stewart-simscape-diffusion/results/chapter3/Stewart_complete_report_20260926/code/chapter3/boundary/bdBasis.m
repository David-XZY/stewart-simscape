function [B,D,DD]=bdBasis(t,T,n)
% General clamped quintic basis. Same endpoint convention as dpBasis.
assert(n>=6 && T>0 && all(isfinite(t(:))),'BD:Basis','Invalid spline.');
u=max(0,min(1,t(:).'/T)); k=[zeros(1,6),(1:n-6)/(n-5),ones(1,6)];
B=values(k,5,u); G=zeros(n-1,n);
for i=1:n-1, G(i,i:i+1)=[-5,5]/(k(i+6)-k(i+1)); end
dk=k(2:end-1); H=zeros(n-2,n-1);
for i=1:n-2, H(i,i:i+1)=[-4,4]/(dk(i+5)-dk(i+1)); end
D=G.'*values(dk,4,u)/T; DD=G.'*H.'*values(dk(2:end-1),3,u)/T^2;
end
function B=values(k,p,u)
last=u==1; u(last)=1-eps; B=zeros(numel(k)-1,numel(u));
for i=1:size(B,1), B(i,:)=u>=k(i)&u<k(i+1); end
for d=1:p
    C=zeros(size(B,1)-1,numel(u));
    for i=1:size(C,1)
        l=k(i+d)-k(i); r=k(i+d+1)-k(i+1);
        if l>0, C(i,:)=C(i,:)+(u-k(i))/l.*B(i,:); end
        if r>0, C(i,:)=C(i,:)+(k(i+d+1)-u)/r.*B(i+1,:); end
    end
    B=C;
end
B(:,last)=0; B(end,last)=1;
end
