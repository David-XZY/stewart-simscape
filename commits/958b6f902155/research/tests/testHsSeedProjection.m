function report=testHsSeedProjection()
task=sampleTransferTask(19,'train'); s=task.scene; p=buildDynamicsProvider(task.model);
d=buildTwoPhaseIHSDiscretization(s,10,5);
[~,original]=buildInitialGuessTwoPhaseIHSImplicit(task.model,s,d);
[initial,z,report]=projectAbfdHsSeed(original,s,d,p);
assert(isequal(initial.Xnode,original.Xnode),'Node position and velocity changed.');
X=initial.Xnode; Xm=initial.XmidIHS; f=[X(7:12,:);initial.Anode]; fm=[Xm(7:12,:);initial.Amid];
mid=Xm-.5*(X(:,1:end-1)+X(:,2:end))-d.h/8*(f(:,1:end-1)-f(:,2:end));
hs=X(:,2:end)-X(:,1:end-1)-d.h/6*(f(:,1:end-1)+4*fm+f(:,2:end));
report.maxHsResidual=max(abs([mid(:);hs(:)])); assert(report.maxHsResidual<1e-12);
assert(max(abs(initial.Anode(4:6,d.waypointNodeIndex:end)),[],'all')<1e-10);
line=s.box.R_S*(s.phase.pCylinderGoal_B-s.phase.pCylinderWaypoint_B);
direction=line/norm(line); transverse=eye(3)-direction*direction.';
assert(max(abs(transverse*initial.Anode(1:3,d.waypointNodeIndex:end)),[],'all')<1e-10);
for k=1:d.numNodes
    actual=p.forward(X(:,k),initial.Fnode(:,k)); assert(norm(actual-f(:,k))<1e-8);
end
for k=1:d.numMidpoints
    actual=p.forward(Xm(:,k),initial.Fmid(:,k)); assert(norm(actual-fm(:,k))<1e-8);
end
research=struct('provider',p,'config',makeAbfdConfig(),'center',[], ...
    'design',[],'buildOnly',true,'enforceInsertionDerivatives',true);
nlp=buildCasadiTrackableIHSNLP(task.model,s,d,initial,struct(),research);
[~,eq]=nlp.eval(z); assert(max(abs(full(eq)))<1e-8);
% A constant acceleration mode is invisible to HS q/v constraints but creates
% angular motion between them. The new independent anchor must reject it.
offset=[0;0;0;.03;0;0]; An=initial.Anode+offset; Am=initial.Amid-.5*offset;
Fn=initial.Fnode; Fm=initial.Fmid;
for k=1:d.numNodes, Fn(:,k)=p.inverse(X(1:6,k),X(7:12,k),An(:,k)); end
for k=1:d.numMidpoints, Fm(:,k)=p.inverse(Xm(1:6,k),Xm(7:12,k),Am(:,k)); end
bad=packIHSDecisionImplicit(X(:,2:end-1),Xm,An,Am,Fn,Fm,d,initial.separator);
[~,eq]=nlp.eval(bad); eq=full(eq);
assert(max(abs(eq(1:end-5)))<1e-8 && max(abs(eq(end-4:end)))>.02);
report.nullModeRejected=true;
report.passed=true; report.scope='HS and dynamics identity only; no path feasibility claim';
end
