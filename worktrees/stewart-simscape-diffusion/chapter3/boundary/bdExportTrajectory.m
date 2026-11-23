function bdExportTrajectory(bundle,task,file,c)
% Reproducible lightweight numeric plot data; actual trajectories only.
t=linspace(0,task.T,ceil(task.T/.005)+1); tr=bdReconstruct(bundle,t); p=sgPhysics(task.model);
[L,Ld,Ldd,F]=p.point(tr.q,tr.v,tr.a); center=zeros(3,numel(t)); axis=center;
for k=1:numel(t)
    R=rpy2rotmZYX(tr.q(4:6,k)); center(:,k)=tr.q(1:3,k)+R*task.scene.objectCylinder.center_P; axis(:,k)=R*task.scene.objectCylinder.axis_P;
end
fullGap=sgClearanceBatch(tr.q,task); halfTask=task; halfTask.scene.objectCylinder.length=task.scene.objectCylinder.length/2;
halfTask.scene.objectCylinder.center_P=task.scene.objectCylinder.center_P+task.scene.objectCylinder.axis_P*task.scene.objectCylinder.length/4;
frontGap=sgClearanceBatch(tr.q,halfTask);
halfTask.scene.objectCylinder.center_P=task.scene.objectCylinder.center_P-task.scene.objectCylinder.axis_P*task.scene.objectCylinder.length/4;
rearGap=sgClearanceBatch(tr.q,halfTask);
q=bdQuality(bundle,task,c);
dpWriteJson(file,struct('taskId',task.taskId,'T',task.T,'t',t,'q',tr.q,'v',tr.v,'a',tr.a, ...
    'center',center,'axis',axis,'legLength',full(L),'legSpeed',full(Ld),'legAcceleration',full(Ldd),'force',full(F), ...
    'payloadGapLower',fullGap,'frontGapLower',frontGap,'rearGapLower',rearGap,'metrics',q, ...
    'clearanceType','separation lower certificate, not exact distance','obstacles',task.scene.hood.obstacles));
end
