function cp=dpSeed(task,index)
cp=dpBaseline(task); dirs=[0,1,-1,0,0,1,-1,1;0,0,0,1,-1,1,1,-1];
if strcmp(task.family,'hood')
    t=linspace(0,task.T,201); u=min(1,t/(task.T*2/3)); w=max(0,(t-task.T*2/3)/(task.T/3));
    ease=@(v) 10*v.^3-15*v.^4+6*v.^5;
    q=task.q0+(task.scene.qWaypoint-task.q0)*ease(u)+(task.qGoal-task.scene.qWaypoint)*ease(w);
    [B,~,~]=dpBasis(t,task.T); cp(:,4:13)=0; cp(:,4:13)=(q-cp*B)/B(4:13,:);
end
direction=dirs(:,1+mod(index-1,8));
if norm(direction)>0, direction=direction/norm(direction); end
delta=task.qGoal(1:3)-task.q0(1:3); delta=delta/norm(delta);
transverse=cross(delta,[0;0;1]); if norm(transverse)<.1, transverse=cross(delta,[0;1;0]); end
transverse=transverse/norm(transverse); vertical=cross(delta,transverse);
offset=.09*(transverse*direction(1)+vertical*direction(2));
cp(1:3,4:13)=cp(1:3,4:13)+offset*sin(pi*(1:10)/11);
end
