function d=sgDiversity(cp,task)
% Evaluate each trajectory/rotation once; exact SO(3) pairwise angle metric.
n=size(cp,3); xyz=zeros(3,101,n); rot=zeros(9,101,n);
for i=1:n
    tr=sgSpline(cp(:,:,i),linspace(0,task.T,101),task.T); xyz(:,:,i)=tr.q(1:3,:);
    for k=1:101, R=rpy2rotmZYX(tr.q(4:6,k)); rot(:,k,i)=R(:); end
end
P=zeros(n); A=P;
for i=1:n
    for j=i+1:n
        P(i,j)=sqrt(mean(sum((xyz(:,:,i)-xyz(:,:,j)).^2,1))); P(j,i)=P(i,j);
        cosine=(sum(rot(:,:,i).*rot(:,:,j),1)-1)/2;
        A(i,j)=sqrt(mean(acos(max(-1,min(1,cosine))).^2)); A(j,i)=A(i,j);
    end
end
d=struct('positionRmsPairsM',P,'attitudeRmsPairsRad',A,'maxPositionRmsM',max(P(:)), ...
    'maxAttitudeRmsRad',max(A(:)),'meaning','Pairwise geometric differences, not homotopy classes.');
end
