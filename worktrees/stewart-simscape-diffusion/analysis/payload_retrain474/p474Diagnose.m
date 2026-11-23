function p474Diagnose(requestPath)
r=jsondecode(fileread(requestPath));c=pgConfig('payload_retrain474_v1');c.returnPolicy='physical';
rows=r.cases;if ~iscell(rows),rows=num2cell(rows);end
for k=1:numel(rows)
    item=rows{k};file=fullfile(r.output,[item.spec.taskId,'.json']);if isfile(file),continue;end
    task=pgMakeTask(item.spec,c);p=pgPhysics(task.model);m=task.model;
    [L,~,~,F]=p.point([task.q0,task.qGoal],zeros(6,2),zeros(6,2));L=full(L);F=full(F);
    distance=abs(L(:,2)-L(:,1));v=m.actuator.ldotMax(:);a=m.actuator.lddotMax(:);
    limit=2*sqrt(distance./a);long=distance>v.^2./a;limit(long)=distance(long)./v(long)+v(long)./a(long);
    row=struct('taskId',task.taskId,'exclude',false,'classification','feasibility_unknown','reason','', ...
        'endpointPassed',task.endpointPassed,'endpointReason',task.endpointReason,'minimumLegMotionTime',max(limit),'T',task.T, ...
        'endpointLengths',L,'endpointForces',F,'routes',{{}},'candidateEvidence',{{}});
    % A conservative geometry-certification failure is not a collision proof.
    if ~task.endpointPassed&&any(strcmp(task.endpointReason,{'endpoint_leg','endpoint_force','endpoint_singularity'}))
        row.exclude=true;row.classification='necessary_endpoint_constraint_violated';row.reason=task.endpointReason;
    elseif task.T+1e-6<max(limit)
        row.exclude=true;row.classification='necessary_duration_bound_violated';row.reason='rest_to_rest_leg_speed_acceleration_lower_bound';
    elseif ~task.endpointPassed
        row.reason='endpoint_geometry_not_certified_no_infeasibility_proof';
    end
    paths=item.statePaths;if ischar(paths),paths={paths};end
    for j=1:numel(paths)
        saved=load(paths{j},'state');state=saved.state;b=[];
        if ~isempty(state.warm),b=state.warm.bundle;
        elseif ~isempty(state.pool)
            scores=cellfun(@(x)x.rank(end),state.pool);[~,ii]=min(scores);b=state.pool{ii}.bundle;
        end
        if ~isempty(b)
            try,e=p474Metrics(b,task,c);e.source=paths{j};row.candidateEvidence{end+1}=e;
            catch err,row.candidateEvidence{end+1}=struct('source',paths{j},'diagnosticError',[err.identifier,':',err.message]);end
        end
    end
    if ~row.exclude&&task.endpointPassed
        clock=tic;cc=c;cc.intervals=120;cc.maxCpuSeconds=600;cc.maxIter=10000;cc.clockHandle=clock;cc.wallBudget=600;
        try
            sol=p474FeasSolve(task,bdBaseline(task,16),cc);audit=pgValidate(sol.bundle,task,c,clock,600);
            row.routes{end+1}=struct('name','ihsid_120_feasibility','found',audit.passed,'seconds',toc(clock),'status',sol.solverStatus);
            if audit.passed,row.classification='feasible_common_algorithm_failure';row.reason='independent_ihsid_witness';witness=sol.bundle;save(fullfile(r.output,[task.taskId,'_ihsid_witness.mat']),'task','witness');end
        catch err,row.routes{end+1}=struct('name','ihsid_120_feasibility','found',false,'seconds',toc(clock),'error',[err.identifier,':',err.message]);end
        clock=tic;cc=c;cc.controlCount=24;found=false;trials=0;
        try
            for initial=1:8
                if toc(clock)>=600,break;end
                b=struct('kind','spline','cp',bdSeed(task,initial,cc),'T',task.T);trials=trials+1;
                while toc(clock)<590
                    [b,info]=p474Repair(b,task,clock,590,cc);audit=pgValidate(b,task,cc,clock,600);
                    if audit.passed,found=true;break;end
                    if info.exhausted||strcmp(info.reason,'time_slice'),break;end
                end
                if found,break;end
            end
            row.routes{end+1}=struct('name','spline24_feasibility','found',found,'seconds',toc(clock),'starts',trials);
            if found,row.classification='feasible_common_algorithm_failure';row.reason='independent_spline_witness';witness=b;save(fullfile(r.output,[task.taskId,'_spline_witness.mat']),'task','witness');end
        catch err,row.routes{end+1}=struct('name','spline24_feasibility','found',false,'seconds',toc(clock),'error',[err.identifier,':',err.message]);end
    end
    if isempty(row.reason),row.reason='no_independent_witness_found_not_an_infeasibility_proof';end
    dpWriteJson(file,row);fprintf('P474_DIAGNOSE %d/%d %s %s\n',k,numel(rows),task.taskId,row.classification);
end
end
