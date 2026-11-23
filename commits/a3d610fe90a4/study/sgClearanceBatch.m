function [gaps,normals]=sgClearanceBatch(q,task)
% Vectorized enumeration of EXACTLY the legacy support directions.
% Result is a lower bound certificate, not an exact distance or penetration.
p=sgPhysics(task.model); rot=full(p.pose(q)); N=size(q,2);
rx=rot(:,1:3:end); ry=rot(:,2:3:end); rz=rot(:,3:3:end);
obj=task.scene.objectCylinder;
axis=rx*obj.axis_P(1)+ry*obj.axis_P(2)+rz*obj.axis_P(3);
center=q(1:3,:)+rx*obj.center_P(1)+ry*obj.center_P(2)+rz*obj.center_P(3);
obs=task.scene.hood.obstacles; gaps=zeros(numel(obs),N); normals=zeros(3,numel(obs),N);
bits=[-1,-1,-1;1,-1,-1;-1,1,-1;1,1,-1;-1,-1,1;1,-1,1;-1,1,1;1,1,1].';
for j=1:numel(obs)
    o=obs(j); delta=o.center_S-center; transverse=delta-axis.*sum(axis.*delta,1);
    fixed=reshape(repmat(o.R_S,1,N),3,3,N); fixed=permute(fixed,[1,3,2]);
    moving=cat(3,rx,ry,rz); edge=zeros(3,N,3); side=zeros(3,N,8);
    for k=1:3, edge(:,:,k)=cross(axis,repmat(o.R_S(:,k),1,N),1); end
    for k=1:8
        v=delta-o.R_S*(bits(:,k).*o.halfSize); side(:,:,k)=v-axis.*sum(axis.*v,1);
    end
    raw=cat(3,fixed,-fixed,moving,-moving,delta,-delta,transverse,-transverse,edge,-edge,side,-side);
    len=sqrt(sum(raw.^2,1)); valid=len>1e-12; raw=raw./max(len,1e-12);
    M=size(raw,3); flat=reshape(raw,3,[]); aa=repmat(axis,1,M); dd=repmat(delta,1,M);
    axial=sum(aa.*flat,1);
    val=sum(dd.*flat,1)-o.halfSize.'*abs(o.R_S.'*flat)- ...
        .5*obj.length*abs(axial)-obj.radius*sqrt(max(0,1-axial.^2));
    val(~valid(:).')=-inf; val=reshape(val,N,M);
    [gaps(j,:),ix]=max(val,[],2); % MATLAB accepts row LHS for column RHS.
    selected=(0:N-1)+N*(ix.'-1)+1;
    normals(:,j,:)=reshape(flat(:,selected),3,1,N);
end
end
