function report=ch5SpeedTeacherPilotV82()
% Frozen balanced pilot: 6 train + 3 validation tasks, both actuator modes.
% Resume completed result states, retain failures, never screen tasks by outcome.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_pilot_v82');
if ~isfolder(directory),mkdir(directory);end
identityFile=fullfile(directory,'identity.json');assert(isfile(identityFile),'ch5:PilotIdentity','Freeze source hashes before execution.');
training=load(fullfile(root,'chapter5','artifacts','tasks','train.mat'),'tasks');validation=load(fullfile(root,'chapter5','artifacts','tasks','validation.mat'),'tasks');
taskList=[training.tasks([8,12,16,20,24,28]);validation.tasks([4,8,12])];
if ~isfile(fullfile(directory,'tasks.mat'))
    tasks=taskList;save(fullfile(directory,'tasks.mat'),'tasks','-v7');writeJSON(fullfile(directory,'tasks.json'),tasks);
else
    frozen=load(fullfile(directory,'tasks.mat'),'tasks');assert(isequaln(frozen.tasks,taskList),'ch5:TaskFreeze','Task source changed.');
end
gate=jsondecode(fileread(fullfile(fileparts(directory),'baseline_retain_audit_v72.json')));assert(gate.completed,'ch5:BaselineGate','Require baseline regression gate.');
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
model=evalin('base','ch5Runtime.modelName');block=[model,'/Strict CLF-CBF-QP Filter'];oldFunction=get_param(block,'FunctionName');
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved,model,block,oldFunction)); %#ok<NASGU>
report=struct('pid',feature('getpid'),'completed',false,'rows',{{}},'formal',false,'expected_runs',18,'identity_file',identityFile,'collector_revision','v82-string-scalar-path-correction');
for mode=["ideal-force","nonideal-force"]
for ti=1:numel(taskList)
    task=taskList{ti};assert(task.family==4,'ch5:PilotCoverage','All pilot tasks must contain continuous obstacle motion.');
    file=fullfile(directory,sprintf('%s_%s.mat',task.id,strrep(mode,'-','_')));rowFile=strrep(file,'.mat','.json');
    if isfile(rowFile),row=jsondecode(fileread(rowFile));report.rows{end+1}=row;writeJSON(fullfile(directory,'summary.json'),report);continue;end
    r=ch5ReducedPrepareV48(4,mode);r.task=task;r.method="baseline";r.seed=0;r.c.episode_seconds=15;r.speed.stop_on_unsafe=true;
    [drift,G]=r.ad.dynamics(task.q0,task.qd0);r.initialForce=full(G)\(-full(drift));
    stewart=evalin('base','stewart');stewart=initializeStewartPose(stewart,'AP',task.q0(1:3)-r.physical.qHome(1:3),'ARB',ch5Rotation(task.q0(4:6)));
    if mode=="nonideal-force"
        actuator=makeNonidealForceActuator(r.physical,struct('forceRateLimit',r.c.actuator_slew_rate,'forceLagTimeConstant',r.c.actuator_time_constant, ...
            'forceInputDelay',r.c.actuator_delay,'forceDeadzone',ones(6,1)*r.c.actuator_deadzone,'initialForce',r.initialForce));
        stewart.nonidealForceActuator=actuator;assignin('base','nonidealForceActuator',actuator);
    end
    r.journalDirectory=fullfile(directory,sprintf('%s_%s_journal',task.id,strrep(char(mode),'-','_')));if ~isfolder(r.journalDirectory),mkdir(r.journalDirectory);end
    r.algorithm_revision='baseline-v72-moving-teacher-pilot';ch5ReleaseEpisodeData(r.mode,'baseline',r.task);
    assignin('base','stewart',stewart);assignin('base','ch5Runtime',r);set_param(block,'FunctionName','ch5TeacherV72Sfun','Parameters','ch5Runtime');
    row=struct('task',task.id,'split',task.split,'subtype',task.subtype,'mode',char(mode),'status','running','admitted',false,'formal',false,'error','');
    writeJSON(fullfile(directory,'active.json'),row);
    try
        timer=tic;set_param(model,'SimulationCommand','update');update=toc(timer);timer=tic;
        output=sim(model,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wall=toc(timer);
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        save(file,'records','predictions','plans','output','r','-v7');metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');
        row.metrics=metrics;row.wall_seconds=wall;row.update_seconds=update;row.simulation_error=output.ErrorMessage;
        row.admitted=metrics.safe_success&&metrics.simulated_end_seconds>=15-1e-8&&isempty(output.ErrorMessage);
        row.status='complete';if ~isempty(output.ErrorMessage),row.status='simulation_failed';end
        save(file,'row','audit','-append');
    catch ex
        row.status='failed';row.error=getReport(ex,'extended','hyperlinks','off');
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        if isfile(file),save(file,'row','-append');else,save(file,'records','predictions','plans','r','row','-v7');end
    end
    writeJSON(rowFile,row);report.rows{end+1}=row;writeJSON(fullfile(directory,'summary.json'),report);
    fprintf('V80 teacher pilot %s %s status%s admitted%d completed%d/18\n',task.id,mode,row.status,row.admitted,numel(report.rows));
end
end
report.completed=numel(report.rows)==18;report.admitted_runs=sum(cellfun(@(z)z.admitted,report.rows));writeJSON(fullfile(directory,'summary.json'),report);
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
