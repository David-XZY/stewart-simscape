function [U,check,traj,info]=ch5ForceLabelRepairV63(x,previous,raw,goal,o,r,k)
% Distribute the minimum force/rate change over the entire prediction horizon.
% Every accepted label passes the unchanged full nonlinear physical gate.
ch5ForceTerminalV62();U=raw;[check,traj]=ch5ForceAuditV62(x,previous,U,goal,o,r,k,[]);
info=struct('method','whole_horizon_smooth_newton','attempts',{{}},'iterations',0);
if check.accepted,return;end
feedback=ch5ReducedFeedbackRolloutV50(r);K=ch5ReducedFeedbackGain(x,r,k);
for iteration=1:5
    if ~check.finite,break;end
    linear=ch5ForceLinearizeV62(x,previous,U,goal,o,r,k,0);S=linear.S;M=linear.M;nodes=linear.nodes;
    [us,Ju]=k.equilibrium(nodes(1:6,end));us=full(us);Ju=full(Ju);Md=M-[zeros(6,48);M(1:end-6,:)];
    residual=[nodes(7:12,end)/.1;(U(:,end)-us)/100];
    F=[S(7:12,:,end)/.1;(M(end-5:end,:)-Ju*S(1:6,:,end))/100];
    if r.mode=="nonideal-force"
        residual=[residual;(nodes(19:24,end)-us)/100];
        F=[F;(S(19:24,:,end)-Ju*S(1:6,:,end))/100];
    end
    forceJ=M/20;rateJ=Md/2;poseJ=S(1:6,:,end)./[.01*ones(3,1);.03*ones(3,1)];
    W=forceJ'*forceJ+rateJ'*rateJ+.05*(poseJ'*poseJ)+1e-7*eye(48);
    forceError=(U-raw)/20;rateError=diff([previous,U-raw+previous],1,2)/2;
    grad=forceJ'*forceError(:)+rateJ'*rateError(:);unconstrained=-W\grad;
    WF=W\F';metric=F*WF;ridge=1e-10*max(1,norm(metric,inf));
    delta=unconstrained-WF*((metric+ridge*eye(size(F,1)))\(residual+F*unconstrained));
    if any(~isfinite(delta)),break;end
    delta=delta*min(1,.5/max(norm(delta,inf),realmin));best=inf;nextU=U;nextCheck=check;nextTraj=traj;
    for fraction=[1,.5,.25]
        candidate=full(feedback(x,nodes,U,K,reshape(fraction*delta,6,8)));
        [cc,ct]=ch5ForceAuditV62(x,previous,candidate,goal,o,r,k,[]);
        info.attempts{end+1}=struct('iteration',iteration,'fraction',fraction,'check',cc);
        if cc.accepted,U=candidate;check=cc;traj=ct;info.iterations=iteration;return;end
        if cc.finite&&cc.inequality_violation<=1e-6
            [target,~]=k.equilibrium(ct.X(1:6,end));target=full(target);
            merit=norm([ct.X(7:12,end)/.1;(candidate(:,end)-target)/100]);
            if r.mode=="nonideal-force",merit=norm([ct.X(7:12,end)/.1;(candidate(:,end)-target)/100;(ct.X(19:24,end)-target)/100]);end
            if merit<best,best=merit;nextU=candidate;nextCheck=cc;nextTraj=ct;end
        end
    end
    if ~isfinite(best),break;end
    U=nextU;check=nextCheck;traj=nextTraj;info.iterations=iteration;
end
end
