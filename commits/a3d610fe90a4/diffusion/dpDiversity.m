function d=dpDiversity(cp,task)
count=size(cp,3); P=zeros(count); R=P;
for i=1:count
    a=dpSpline(cp(:,:,i),linspace(0,task.T,101),task.T);
    for j=i+1:count
        b=dpSpline(cp(:,:,j),a.t,task.T);
        P(i,j)=sqrt(mean(sum((a.q(1:3,:)-b.q(1:3,:)).^2,1))); P(j,i)=P(i,j);
        angles=zeros(1,101);
        for k=1:101
            ra=rpy2rotmZYX(a.q(4:6,k)); rb=rpy2rotmZYX(b.q(4:6,k));
            angles(k)=acos(max(-1,min(1,(trace(ra.'*rb)-1)/2)));
        end
        R(i,j)=sqrt(mean(angles.^2)); R(j,i)=R(i,j);
    end
end
d=struct('positionRmsPairsM',P,'attitudeRmsPairsRad',R, ...
    'maxPositionRmsM',max(P(:)),'maxAttitudeRmsRad',max(R(:)), ...
    'meaning','Pairwise geometric difference, not homotopy classes.');
end
