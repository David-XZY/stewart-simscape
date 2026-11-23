function [U,qs]=ch5ForceWarmV57(x,previous,saved,r,k,time)
% Append the local terminal controller, then correct the dynamics-generated warm plan.
old=saved.trajectory;shift=round((time-saved.trajectory_time)/r.c.Ts);qs=old.artificial_pose;
assert(shift>=0&&shift<=r.c.H,'ch5:WarmShift','Retained plan expired.');
U=old.U(:,shift+1:end);warm=old.X(:,shift+1:end);state=old.X(:,end);last=old.U(:,end);t=old.terminal;
for j=1:shift
    e=([state;last]-t.xi)./t.scale;u=last-t.Du*t.K*e;
    state=full(r.plant.fd(state,u,zeros(6,1)));U(:,end+1)=u;warm(:,end+1)=state;last=u; %#ok<AGROW>
end
feedback=ch5ReducedFeedbackRolloutV50(r);K=ch5ReducedFeedbackGain(x,r,k);
U=full(feedback(x,warm,U,K,zeros(6,8)));
end
