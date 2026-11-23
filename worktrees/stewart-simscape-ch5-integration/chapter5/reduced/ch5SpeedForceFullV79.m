function report=ch5SpeedForceFullV79()
% Warm, independent short Simscape runs before complete development episodes.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','full_v79');
if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'summary.json')),'ch5:ExistingAttempt','Preserve previous short attempt.');
logicFile=fullfile(fileparts(directory),'online_v79','logic.json');
if ~isfile(logicFile),ch5SpeedForceLogicV79();end
gate=jsondecode(fileread(logicFile));assert(gate.completed&&gate.accepted,'ch5:RawGate','Require complete fixed raw audit and full-DDIM parity.');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
model=evalin('base','ch5Runtime.modelName');block=[model,'/Strict CLF-CBF-QP Filter'];oldFunction=get_param(block,'FunctionName');
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved,model,block,oldFunction)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'rows',{{}},'completed',false,'formal',false,'formal_runs',0,'error','');
cases={1,"nonideal-force",15;4,"nonideal-force",15;1,"nonideal-force",15;1,"ideal-force",15};
try
    for ci=1:size(cases,1)
        index=cases{ci,1};mode=cases{ci,2};stop=cases{ci,3};r=ch5ReducedPrepareV48(index,mode);
        if ci==3
            targetTasks=load(fullfile(root,'chapter5','artifacts','tasks','development.mat'),'tasks');
            r.task.goals=[r.task.goals(:,1),targetTasks.tasks{3}.goals(:,end)];r.task.goal_times=[0,8];
            r.task.family=3;r.task.subtype='terminal_goal_switch';r.task.id='development_terminal_switch_070';
            r.task.seed=70070;r.task.index=703;index=3;
        end
        r.reduced=struct('iterations',1,'backtracking',3,'linearization_collision_reserve',.0002,'velocity_weight',1, ...
            'factor_derivatives',true,'reuse_sensitivities',true,'qp_row_scaling',true,'qp_max_iterations',400,'trust_radius',1, ...
            'audit_raw_candidates',false,'sensitivity_maximum_age',.3);
        r.force_weights=load(fullfile(root,'chapter5','artifacts','control_force_v62','raw_v76','seed_5101','weights.mat'));
        r.force_weights.checkpoint_file=fullfile(fileparts(directory),'raw_v76','seed_5101','model.pt');r.algorithm_revision='force-v79-objective-ranked-admission-development';r.c.episode_seconds=stop;r.speed.stop_on_unsafe=true;
        ch5ReleaseEpisodeData(r.mode,'diffusion',r.task);timer=tic;k=ch5ForceKernelsV64(r);ch5ForceTerminalV62();
        brake=ch5ReducedBrakeSequence(r);x=[r.task.q0;r.task.qd0;repmat(r.initialForce,(r.plant.n-12)/6,1)];U=full(brake(x,r.initialForce));
        [~,traj]=ch5ForceAuditV77(x,r.initialForce,U,r.task.goals(:,1),zeros(22,8),r,k,[]);
        [~,o]=ch5Observe(r.task,0);prepared=ch5ForcePrepareStepV77(x,r.initialForce,U,r.task.goals(:,1),o,U,r,k,0,traj.artificial_pose);
        ch5ForceSolveStepV77(prepared,U,[]);
        ch5ForceSampleV76(ch5Context(x,r.initialForce,r.task.goals(:,1),zeros(22,8),r.mode),r.initialForce,r.force_weights,zeros(360,2),r,k);
        build=toc(timer);assignin('base','ch5Runtime',r);set_param(block,'FunctionName','ch5ForceV79Sfun','Parameters','ch5Runtime');
        timer=tic;set_param(r.modelName,'SimulationCommand','update');update=toc(timer);
        timer=tic;output=sim(r.modelName,'SimulationMode','normal','StopTime',num2str(stop),'ReturnWorkspaceOutputs','on','CaptureErrors','on');wall=toc(timer);
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        file=fullfile(directory,sprintf('%03d_%s.mat',index,strrep(mode,'-','_')));save(file,'records','predictions','plans','output','r','-v7');
        metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');times=cellfun(@(z)z.controller_seconds,records);
        refresh=cellfun(@(z)z.prediction.refresh,predictions);fresh=0;reused=0;
        for pi=1:numel(predictions)
            aa=predictions{pi}.prediction.denoise_attempts;
            if ~isempty(aa)&&isfield(aa{1},'linearization_phases')
                % Stage counts are stored on every solve of the shared group.
                if isfield(aa{1},'sensitivity'),fresh=fresh+aa{1}.sensitivity.fresh_stages;reused=reused+aa{1}.sensitivity.reused_stages;end
            end
        end
        row=struct('task',index,'mode',char(mode),'duration',stop,'metrics',metrics,'wall_seconds',wall,'warmup_seconds',build,'update_seconds',update, ...
            'refresh_seconds',times(refresh),'execution_seconds',times(~refresh),'simulation_error',output.ErrorMessage,'fresh_stages',fresh,'reused_stages',reused,'formal',false);
        row.generated_count=sum(cellfun(@(z)fieldDefault(z.prediction,'raw_generated_count',0),predictions));
        row.refresh_stats=struct('median',median(times(refresh)),'p95',prctile(times(refresh),95),'maximum',max(times(refresh)));
        row.raw_audited=sum(cellfun(@(z)z.prediction.raw_audited,predictions));row.raw_accepted=sum(cellfun(@(z)z.prediction.raw_accepted,predictions));
        save(file,'row','audit','-append');report.rows{end+1}=row;writeJSON(strrep(file,'.mat','.json'),row);writeJSON(fullfile(directory,'summary.json'),report);
        fprintf('V79 objective full%d %s safe%d arrival%d pos%.3fmm wall%.2fs refresh%.3fs\n',index,mode,metrics.all_constraints_valid,metrics.arrival,metrics.final_position*1000,wall,median(times(refresh)));
        if ~isempty(output.ErrorMessage)||~metrics.all_constraints_valid,report.error='Short development physical check failed.';break;end
    end
    report.completed=numel(report.rows)==size(cases,1);writeJSON(fullfile(directory,'summary.json'),report);
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');writeJSON(fullfile(directory,'summary.json'),report);rethrow(ex);end
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end

function value=fieldDefault(s,name,default)
if isfield(s,name),value=s.(name);else,value=default;end
end
