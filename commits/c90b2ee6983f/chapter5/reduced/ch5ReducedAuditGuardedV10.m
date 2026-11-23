function [check,trajectory,cost]=ch5ReducedAuditGuardedV10(x,previous,U,goal,o,r,k,prior)
% Reject exact algebraic input violations before the expensive true rollout.
D=diff([previous,U],1,2);finite=all(isfinite([U(:);previous]));
forceViolation=max([reshape(r.strict.forceMin-U,[],1);reshape(U-r.strict.forceMax,[],1);0]);
rateViolation=max([reshape(abs(D)/r.c.Ts-r.strict.forceRateLimit,[],1);0]);
if ~finite||forceViolation>1e-6||rateViolation>1e-6
    check=struct('accepted',false,'finite',finite,'equality_tolerance_ratio',0, ...
        'terminal_checked',false,'inequality_violation',forceViolation,'command_rate_violation',rateViolation, ...
        'scope','full_horizon','cheap_reject',true,'rejection_reason','algebraic_input_constraint');
    trajectory=struct('X',[],'U',U,'delta',D);cost=inf;return;
end
[check,trajectory,cost]=ch5ReducedAuditFastV8(x,previous,U,goal,o,r,k,prior);
check.scope='full_horizon';check.terminal_checked=true;check.cheap_reject=false;
end
