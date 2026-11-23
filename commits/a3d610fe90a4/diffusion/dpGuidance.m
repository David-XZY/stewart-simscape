function fun=dpGuidance(task,asset,c)
% Gradient is with respect to standardized free control offsets.
import casadi.*
p=dpPhysics(task.model); m=task.model; z=SX.sym('z',60);
base=dpBaseline(task); cp=[base(:,1:3),base(:,4:13)+reshape(z.*asset.outputScale(:)+asset.outputMean(:),6,10),base(:,14:16)];
[B,D,DD]=dpBasis(linspace(0,task.T,61),task.T); Q=cp*B; V=cp*D; A=cp*DD;
[L,Ld,Ldd,F,~,~,phi]=p.point(Q,V,A);
kin=sum1(sum2(fmax(0,(L-m.lmax)/.1).^2+fmax(0,(m.lmin-L)/.1).^2+ ...
    fmax(0,abs(Ld)/.6-1).^2+fmax(0,abs(Ldd)/1.2-1).^2))/(6*size(Q,2));
fr=sum1(sum2(fmax(0,abs(F)/2000-1).^2))/(6*size(Q,2));
collision=SX(0); obs=task.scene.hood.obstacles;
for i=1:size(Q,2)
    R=p.pose(Q(:,i)); center=Q(1:3,i)+R*task.scene.objectCylinder.center_P;
    for j=1:numel(obs)
        delta=obs(j).center_S-center; direction=delta/sqrt(delta.'*delta+1e-20);
        axis=R*task.scene.objectCylinder.axis_P; transverse=delta-axis*(axis.'*delta);
        transverse=transverse/sqrt(transverse.'*transverse+1e-20);
        edge=cross(repmat(axis,1,3),obs(j).R_S,1); edge=edge./repmat(sqrt(sum1(edge.^2)+1e-20),3,1);
        normals=[obs(j).R_S,-obs(j).R_S,R,-R,direction,-direction,transverse,-transverse,edge,-edge]; gap=SX(-1e3);
        for k=1:size(normals,2), gap=fmax(gap,dpGapExpr(Q(:,i),normals(:,k),obs(j),task,p)); end
        collision=collision+fmax(0,(.01-gap)/.01)^2/(size(Q,2)*numel(obs));
    end
end
scale=[.1;.1;.1;deg2rad(10)*ones(3,1)];
curvature=diff(cp,2,2)./repmat(scale,1,size(cp,2)-2);
smooth=sum1(sum2(curvature.^2))/numel(curvature);
parts=[collision;kin;fr;sum2(phi)/size(Q,2);smooth]; energy=c.guidanceWeights*parts;
fun=Function('dp_guidance',{z},{energy,gradient(energy,z),parts});
end
