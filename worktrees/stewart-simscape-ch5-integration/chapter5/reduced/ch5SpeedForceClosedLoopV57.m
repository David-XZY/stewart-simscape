function report=ch5SpeedForceClosedLoopV57()
% Independent development Simscape runs. No formal task or old checkpoint is modified.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v57','pilot','closed_loop');
if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'summary.json')),'ch5:ExistingAttempt','Retain existing closed-loop attempts.');
parity=load(fullfile(fileparts(directory),'candidate_report.mat'),'report');
assert(parity.report.completed&&max([parity.report.parity.v_max,parity.report.parity.sample_max])<1e-4,'ch5:UnverifiedWeights','Require completed inference parity and candidate audit.');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
model=evalin('base','ch5Runtime.modelName');block=[model,'/Strict CLF-CBF-QP Filter'];oldFunction=get_param(block,'FunctionName');
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved,model,block,oldFunction)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'rows',{{}},'completed',false,'formal_ready',false,'formal_runs',0,'error','');
cases={1,"nonideal-force";4,"nonideal-force";3,"nonideal-force";1,"ideal-force";4,"ideal-force";3,"ideal-force"};
try
    for ci=1:size(cases,1)
        index=cases{ci,1};mode=cases{ci,2};r=ch5ReducedPrepareV48(index,mode);
        r.reduced=struct('iterations',1,'backtracking',3,'linearization_collision_reserve',.0002,'velocity_weight',1, ...
            'factor_derivatives',false,'reuse_sensitivities',false,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1);
        r.force_weights=load(fullfile(fileparts(directory),'seed_5101','weights.mat'));r.algorithm_revision='force-v57-development';
        r.c.episode_seconds=15;r.speed.stop_on_unsafe=true;
        ch5ReleaseEpisodeData(r.mode,'diffusion',r.task);timer=tic;k=ch5ReducedKernelsV4(r);
        ch5ReducedFeedbackRolloutV50(r);brake=ch5ReducedBrakeSequence(r);x=[r.task.q0;r.task.qd0;repmat(r.initialForce,(r.plant.n-12)/6,1)];
        U=full(brake(x,r.initialForce));[check,traj]=ch5ForceAuditV57(x,r.initialForce,U,r.task.goals(:,1),zeros(22,8),r,k,[]);
        s=struct('trajectory',traj,'trajectory_time',0);ch5ForceWarmV57(x,r.initialForce,s,r,k,r.c.Ts);
        ch5ForceSampleV57(ch5Context(x,r.initialForce,r.task.goals(:,1),zeros(22,8),r.mode),r.initialForce,r.force_weights,zeros(360,4));
        [~,o]=ch5Observe(r.task,0);ch5ReducedHoldAuditV22(x,r.initialForce,r.initialForce,o,r);
        ch5ForceStepV57(x,r.initialForce,U,r.task.goals(:,1),o,repmat(U,1,1,4),r,k,0,traj.artificial_pose);
        buildSeconds=toc(timer);assignin('base','ch5Runtime',r);
        set_param(block,'FunctionName','ch5ForceV57Sfun','Parameters','ch5Runtime');
        update=tic;set_param(r.modelName,'SimulationCommand','update');updateSeconds=toc(update);
        fprintf('V57 Simscape start task%d %s, warmup %.2fs update %.2fs\n',index,mode,buildSeconds,updateSeconds);
        timer=tic;output=sim(r.modelName,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wallSeconds=toc(timer);
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        file=fullfile(directory,sprintf('%03d_%s.mat',index,strrep(mode,'-','_')));save(file,'records','predictions','plans','output','r','-v7');
        metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');
        times=cellfun(@(z)z.controller_seconds,records);refresh=cellfun(@(z)z.prediction.refresh,predictions);
        row=struct('task',index,'mode',char(mode),'metrics',metrics,'wall_seconds',wallSeconds,'build_seconds',buildSeconds,'update_seconds',updateSeconds, ...
            'simulation_error',output.ErrorMessage,'refresh_seconds',times(refresh),'execution_seconds',times(~refresh), ...
            'refresh_deadline_misses',nnz(times(refresh)>.1),'execution_deadline_misses',nnz(times(~refresh)>.02), ...
            'raw_accepted',sum(cellfun(@(z)z.prediction.raw_accepted,predictions)), ...
            'projected_accepted',sum(cellfun(@(z)z.prediction.projected_accepted,predictions)), ...
            'terminal_certified',false,'formal',false);
        save(file,'row','audit','-append');writeJSON(strrep(file,'.mat','.json'),row);report.rows{end+1}=row;writeJSON(fullfile(directory,'summary.json'),report);
        fprintf('V57 task%d %s safe%d arrival%d final %.3fmm fallback%d QPfailed%d wall%.2fs\n',index,mode,metrics.all_constraints_valid,metrics.arrival,metrics.final_position*1000,metrics.fallback_count,metrics.predictive_solver_failures,wallSeconds);
        if ~isempty(output.ErrorMessage)||~metrics.safe_success,report.error='Development acceptance failed; remaining cases withheld.';break;end
    end
    report.completed=size(cases,1)==numel(report.rows);writeJSON(fullfile(directory,'summary.json'),report);
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');writeJSON(fullfile(directory,'summary.json'),report);rethrow(ex);end
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
