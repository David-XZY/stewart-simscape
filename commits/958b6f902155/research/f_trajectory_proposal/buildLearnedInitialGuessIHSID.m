function [z,initial,extra]=buildLearnedInitialGuessIHSID(cp,model,scene,disc,provider)
[~,initial]=buildInitialGuessTwoPhaseIHSImplicit(model,scene,disc);
X=[initial.Xnode,[initial.QmidCurve;initial.VmidCurve]];
A=[initial.Anode,initial.Amid]; times=[disc.tNode,disc.tMid];
ids=times<=scene.phase.durationApproach+1e-10;
[q,v,a]=evaluateTrajectorySpline(cp,times(ids),scene.phase.durationApproach);
X(:,ids)=[q;v]; A(:,ids)=a; F=zeros(6,numel(times));
for k=1:numel(times), F(:,k)=provider.inverse(X(1:6,k),X(7:12,k),A(:,k)); end
n=disc.numNodes;
[sep,extra]=buildAbfdSeparators(X(1:6,1:n),X(1:6,n+1:end),scene,disc);
external=struct('Xnode',X(:,1:n),'Xmid',X(:,n+1:end),'Anode',A(:,1:n), ...
    'Amid',A(:,n+1:end),'Fnode',F(:,1:n),'Fmid',F(:,n+1:end),'separator',sep);
[z,initial]=buildInitialGuessTwoPhaseIHSImplicit(model,scene,disc,external);
z=[z;extra(:)];
end
