function plan=ch5SpeedIhsidV66(q0,v0,goal,o,runtime,duration,time)
% Keep the original collocation graph, four starts and warm lanes alive.
timer=tic;bound=ch5IhsidReachabilityV66(q0,v0,goal,duration,runtime);precheck=toc(timer);
if bound.infeasible
    attempts=cell(4,1);
    for start=1:4
        attempts{start}=struct('start',start,'status','fixed_duration_unreachable','solver_success',false, ...
            'solver_called',false,'seconds',0,'reachability',bound);
    end
    plan=struct('accepted',false,'attempts',{attempts},'seconds',precheck,'initial_state',[q0;v0], ...
        'goal',goal,'precheck_infeasible',true);return;
end
plan=ch5SpeedIhsid(q0,v0,goal,o,runtime,duration,time);
plan.reachability_precheck=bound;plan.reachability_precheck_seconds=precheck;
end
