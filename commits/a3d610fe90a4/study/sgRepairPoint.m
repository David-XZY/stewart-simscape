function fun=sgRepairPoint(task)
% One-state AD graph, mapped over time; obstacle support planes are parameters.
% Normals are fixed during each line search, refreshed between iterations.
persistent savedKeys saved;
key=dpHash(struct('model',task.model,'object',task.scene.objectCylinder));
if isempty(savedKeys), savedKeys={}; saved={}; end
k=find(strcmp(savedKeys,key),1); if ~isempty(k), fun=saved{k}; return; end
import casadi.*
q=SX.sym('q',6); v=SX.sym('v',6); a=SX.sym('a',6);
n=SX.sym('normals',3,6); centers=SX.sym('centers',3,6); supports=SX.sym('supports',1,6); mask=SX.sym('mask',1,6);
p=sgPhysics(task.model); m=task.model; obj=task.scene.objectCylinder;
[L,Ld,Ldd,F]=p.point(q,v,a); R=p.pose(q); center=q(1:3)+R*obj.center_P; axis=R*obj.axis_P;
kin=sum1(fmax(0,(m.lmin-L)/.1).^2+fmax(0,(L-m.lmax)/.1).^2+ ...
    fmax(0,abs(Ld)./m.actuator.ldotMax-1).^2+fmax(0,abs(Ldd)./m.actuator.lddotMax-1).^2)/6;
force=sum1(fmax(0,(m.actuator.forceMin-F)/2000).^2+fmax(0,(F-m.actuator.forceMax)/2000).^2)/6;
collision=SX(0);
for j=1:6
    axial=axis.'*n(:,j); radial=n(:,j)-axis*axial;
    gap=n(:,j).'*(centers(:,j)-center)-supports(j)-obj.length/2*abs(axial)-obj.radius*sqrt(radial.'*radial+1e-12);
    collision=collision+mask(j)*fmax(0,(.01-gap)/.01)^2;
end
collision=collision/fmax(1,sum2(mask)); energy=10*collision+kin+force;
fun=Function('sg_repair_point',{q,v,a,n,centers,supports,mask},{energy,gradient(energy,[q;v;a])});
savedKeys=[savedKeys,{key}]; saved=[saved,{fun}];
if numel(savedKeys)>8, savedKeys=savedKeys(end-7:end); saved=saved(end-7:end); end
end
