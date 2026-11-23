function [initial,z,report]=projectAbfdHsSeed(initial,scene,disc,provider)
% Exact linear HS consistency projection, preserving every node q and qdot.
% Anchor the waypoint acceleration at zero so insertion accelerations remain
% tangent to its analytic line with zero angular acceleration between nodes.
% This is a warm-start operation; force/path/dense acceptance remain mandatory.
X=initial.Xnode; q=X(1:6,:); v=X(7:12,:); h=disc.h;
qm=.5*(q(:,1:end-1)+q(:,2:end))+h/8*(v(:,1:end-1)-v(:,2:end));
vm=1.5/h*(q(:,2:end)-q(:,1:end-1))-.25*(v(:,1:end-1)+v(:,2:end));
difference=8/h*(vm-.5*(v(:,1:end-1)+v(:,2:end)));
relative=[zeros(6,1),-cumsum(difference,2)];
offset=-relative(:,disc.waypointNodeIndex); An=relative+offset;
Am=1.5/h*(v(:,2:end)-v(:,1:end-1))-.25*(An(:,1:end-1)+An(:,2:end));
report=struct('maxMidPositionChange',max(abs(qm-initial.XmidIHS(1:6,:)),[],'all'), ...
    'maxNodeAccelerationChange',max(abs(An-initial.Anode),[],'all'),'fullTrajectoryAccepted',false);
initial.XmidIHS=[qm;vm]; initial.Xmid=initial.XmidIHS;
initial.QmidCurve=qm; initial.VmidCurve=vm; initial.Anode=An; initial.Amid=Am;
for k=1:disc.numNodes, initial.Fnode(:,k)=provider.inverse(q(:,k),v(:,k),An(:,k)); end
for k=1:disc.numMidpoints, initial.Fmid(:,k)=provider.inverse(qm(:,k),vm(:,k),Am(:,k)); end
[initial.separator,extra]=buildAbfdSeparators(q,qm,scene,disc);
z=packIHSDecisionImplicit(X(:,2:end-1),initial.XmidIHS,An,Am,initial.Fnode,initial.Fmid,disc,initial.separator);
z=[z;extra(:)];
end
