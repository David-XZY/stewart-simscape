function report=ch5SpeedTeacherRepairV91()
% Fixed development retests of the crossing failure and an orbit in both modes.
% No training admission until a complete physical closed-loop check succeeds.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_repair_v91');
if ~isfolder(directory),mkdir(directory);end
assert(isfile(fullfile(directory,'identity.json')),'ch5:IdentityGate','Freeze sources before execution.');
parent=fileparts(directory);gate=jsondecode(fileread(fullfile(parent,'teacher_failure_v83','restart_audit_v89.json')));
assert(gate.completed&&gate.all_original_hard_constraints_preserved,'ch5:RestartGate','Require an audited final-QP repair.');
source=load(fullfile(parent,'teacher_pilot_v82','tasks.mat'),'tasks');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
model=evalin('base','ch5Runtime.modelName');block=[model,'/Strict CLF-CBF-QP Filter'];oldFunction=get_param(block,'FunctionName');
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved,model,block,oldFunction)); %#ok<NASGU>
ids={'train_016','train_012','train_012'};modes={'nonideal-force','ideal-force','nonideal-force'};
report=struct('pid',feature('getpid'),'completed',false,'formal',false,'expected_runs',3,'rows',{{}}, ...
    'revision','v91-retreat-reuse-and-causal-acceleration-reserve');
for ci=1:numel(ids)
    task=source.tasks{find(cellfun(@(z)strcmp(z.id,ids{ci}),source.tasks),1)};mode=string(modes{ci});
    file=fullfile(directory,sprintf('%s_%s.mat',task.id,strrep(char(mode),'-','_')));rowFile=strrep(file,'.mat','.json');
    if isfile(rowFile),report.rows{end+1}=jsondecode(fileread(rowFile));continue;end
    r=ch5ReducedPrepareV48(4,mode);r.task=task;r.method="baseline";r.seed=0;r.c.episode_seconds=15;r.speed.stop_on_unsafe=true;
    [drift,G]=r.ad.dynamics(task.q0,task.qd0);r.initialForce=full(G)\(-full(drift));
    stewart=evalin('base','stewart');stewart=initializeStewartPose(stewart,'AP',task.q0(1:3)-r.physical.qHome(1:3),'ARB',ch5Rotation(task.q0(4:6)));
    if mode=="nonideal-force"
        actuator=makeNonidealForceActuator(r.physical,struct('forceRateLimit',r.c.actuator_slew_rate,'forceLagTimeConstant',r.c.actuator_time_constant, ...
            'forceInputDelay',r.c.actuator_delay,'forceDeadzone',ones(6,1)*r.c.actuator_deadzone,'initialForce',r.initialForce));
        stewart.nonidealForceActuator=actuator;assignin('base','nonidealForceActuator',actuator);
    end
    r.algorithm_revision=report.revision;r.journalDirectory=fullfile(directory,sprintf('%s_%s_journal',task.id,strrep(char(mode),'-','_')));
    if ~isfolder(r.journalDirectory),mkdir(r.journalDirectory);end
    ch5ReleaseEpisodeData(r.mode,'baseline',r.task);assignin('base','stewart',stewart);assignin('base','ch5Runtime',r);
    set_param(block,'FunctionName','ch5TeacherV91Sfun','Parameters','ch5Runtime');
    row=struct('task',task.id,'split',task.split,'subtype',task.subtype,'mode',char(mode),'status','running','admitted',false,'formal',false,'error','');
    writeJSON(fullfile(directory,'active.json'),row);
    try
        timer=tic;set_param(model,'SimulationCommand','update');update=toc(timer);timer=tic;
        output=sim(model,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wall=toc(timer);
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        save(file,'records','predictions','plans','output','r','-v7');metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');
        row.metrics=metrics;row.wall_seconds=wall;row.update_seconds=update;row.simulation_error=output.ErrorMessage;
        row.admitted=metrics.safe_success&&metrics.simulated_end_seconds>=15-1e-8&&isempty(output.ErrorMessage);
        row.restart_count=sum(cellfun(@(z)isfield(z.diagnostic,'feasible_seed_restart')&&z.diagnostic.feasible_seed_restart,records));
        row.status='complete';if ~isempty(output.ErrorMessage),row.status='simulation_failed';end
        planning_stats=ch5TeacherPlanningStatsV90(plans,records);row.planning=planning_stats;
        save(file,'row','audit','planning_stats','-append');
    catch ex
        row.status='failed';row.error=getReport(ex,'extended','hyperlinks','off');
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        save(file,'records','predictions','plans','r','row','-v7');
    end
    writeJSON(rowFile,row);report.rows{end+1}=row;writeJSON(fullfile(directory,'summary.json'),report);
    fprintf('V91 full repair %s %s status%s admitted%d results%d/3\n',task.id,mode,row.status,row.admitted,numel(report.rows));
    if ~row.admitted,report.stopped_on_failed_acceptance=true;writeJSON(fullfile(directory,'summary.json'),report);return;end
end
report.completed=numel(report.rows)==3;report.admitted_runs=sum(cellfun(@(z)z.admitted,report.rows));
writeJSON(fullfile(directory,'summary.json'),report);writeJSON(fullfile(directory,'active.json'),struct('status','complete','results',numel(report.rows)));
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
