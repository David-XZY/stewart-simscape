function [base,extra]=buildAbfdSeparators(Q,Qmid,scene,disc)
allQ=[Q,Qmid]; normals=cell(1,size(allQ,2));
for k=1:size(allQ,2)
    d=evaluateCylinderBoxDistanceNumeric(allQ(:,k),scene); normals{k}=d.bestNormals;
end
base=zeros(8,disc.numCollisionCertificates); cursor=0;
indices=[1:disc.numIntervalsApproach+1,disc.numNodes+(1:disc.numIntervalsApproach)];
for k=indices
    cursor=cursor+1; base(:,cursor)=at(allQ(:,k),normals{k}(:,1),scene.hood.obstacles(1),scene);
end
for j=2:3
    for k=1:size(allQ,2)
        cursor=cursor+1; base(:,cursor)=at(allQ(:,k),normals{k}(:,j),scene.hood.obstacles(j),scene);
    end
end
extra=zeros(8,max(0,numel(scene.hood.obstacles)-3)*size(allQ,2)); cursor=0;
for j=4:numel(scene.hood.obstacles)
    for k=1:size(allQ,2)
        cursor=cursor+1; extra(:,cursor)=at(allQ(:,k),normals{k}(:,j),scene.hood.obstacles(j),scene);
    end
end
end
function s=at(q,n,obstacle,scene)
R=rpy2rotmZYX(q(4:6)); axis=R*scene.objectCylinder.axis_P;
s=[n;abs(obstacle.R_S.'*n);abs(n.'*axis);sqrt(max(0,1-(n.'*axis)^2)+scene.collision.smoothingEps^2)];
end
