function [u,d] = ch5SafetyQp(q,qd,qr,vr,ar,nominal,previous,o,runtime,time)
cfg=runtime.strict;cfg.time=time;
ev=evaluateStrictClfCbfAdModel(q,qd,runtime.physical,runtime.scene,cfg);
[clf,cbf]=buildStrictClfCbfConstraints(q,qd,qr,vr,ar,runtime.physical,runtime.scene,cfg,ev);
% Replace only the original three task-specific collision rows.
keep=~startsWith(cbf.names,'collision_');cbf.A=cbf.A(keep,:);cbf.b=cbf.b(keep);cbf.names=cbf.names(keep);
keep=~startsWith(cbf.stateNames,'collision_');cbf.stateNames=cbf.stateNames(keep);cbf.stateMargins=cbf.stateMargins(keep);
keep=~startsWith(cbf.psi1Names,'collision_');cbf.psi1Names=cbf.psi1Names(keep);cbf.psi1=cbf.psi1(keep);
[gap,grad,ht,hess]=runtime.collision.cbf(q,qd,o);
gap=full(gap);grad=full(grad);ht=full(ht);hess=full(hess);
for i=find(o(22,:)>.5)
    h=gap(i)-runtime.c.safe_distance-runtime.c.sample_hold_margin;
    hd=grad(i,:)*qd+ht(i);alpha=12;
    cbf.A(end+1,:)=-grad(i,:)*ev.inputMap;
    cbf.b(end+1,1)=grad(i,:)*ev.drift+hess(i)+2*alpha*hd+alpha^2*h;
    name="collision_"+i;cbf.names(end+1)=name;cbf.stateNames(end+1)=name;cbf.stateMargins(end+1)=h;
    cbf.psi1Names(end+1)=name;cbf.psi1(end+1)=hd+alpha*h;
end
cbf.minimumStateMargin=min(cbf.stateMargins);cbf.minimumPsi1=min(cbf.psi1);
[u,d]=solveStrictClfCbfQp(nominal,previous,clf,cbf,cfg);
d.cbf=cbf;d.clf=clf;
end
