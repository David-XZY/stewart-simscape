function stats=ch5TeacherPlanningStatsV90(plans,records)
% Count NLP invocations separately from four-start precheck-only records.
stats=struct('planning_checks',numel(plans),'ihsid_calls',0,'primary_calls',0,'backup_calls',0, ...
    'start_records',0,'solver_calls',0,'precheck_only_starts',0,'solver_failures',0, ...
    'postcheck_rejections',0,'solver_seconds',0,'planning_seconds',0,'retreat_reuse_checks',0,'calls',{{}});
for j=1:numel(records),stats.planning_seconds=stats.planning_seconds+records{j}.planning_seconds;end
for j=1:numel(plans)
    entry=plans{j};
    if strcmp(entry.reference_decision,'retained_checked_retreat_while_goal_hold_obstructed')
        stats.retreat_reuse_checks=stats.retreat_reuse_checks+1;
    end
    for field={'plan','backup_plan'}
        name=field{1};if ~isfield(entry,name)||isempty(entry.(name)),continue;end
        p=entry.(name);if ~isfield(p,'attempts')||isempty(p.attempts),continue;end
        stats.ihsid_calls=stats.ihsid_calls+1;
        if strcmp(name,'plan'),stats.primary_calls=stats.primary_calls+1;else,stats.backup_calls=stats.backup_calls+1;end
        call=struct('time',entry.time,'purpose',name,'accepted',p.accepted,'solver_calls',0,'precheck_only_starts',0,'seconds',p.seconds);
        for a=1:numel(p.attempts)
            attempt=p.attempts{a};called=~isfield(attempt,'solver_called')||attempt.solver_called;
            stats.start_records=stats.start_records+1;
            if called
                stats.solver_calls=stats.solver_calls+1;call.solver_calls=call.solver_calls+1;
                success=isfield(attempt,'solver_success')&&attempt.solver_success;
                stats.solver_failures=stats.solver_failures+double(~success);
                if success&&isfield(attempt,'postcheck')&&~attempt.postcheck.accepted
                    stats.postcheck_rejections=stats.postcheck_rejections+1;
                end
                if isfield(attempt,'solver_seconds'),stats.solver_seconds=stats.solver_seconds+attempt.solver_seconds;end
            else
                stats.precheck_only_starts=stats.precheck_only_starts+1;call.precheck_only_starts=call.precheck_only_starts+1;
            end
        end
        stats.calls{end+1}=call;
    end
end
end
