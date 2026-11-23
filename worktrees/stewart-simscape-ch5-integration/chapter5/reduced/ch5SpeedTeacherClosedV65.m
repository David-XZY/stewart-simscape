function report=ch5SpeedTeacherClosedV65()
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','control_force_v62','teacher_v65');
if ~isfolder(directory),mkdir(directory);end
assert(~isfile(fullfile(directory,'summary.json')),'ch5:ExistingAttempt','Preserve teacher attempts.');
gate=load(fullfile(fileparts(directory),'teacher_interval_audit.mat'),'report');assert(gate.report.completed);
names={'ch5Runtime','stewart','controller','nonidealForceActuator'};saved=cell(size(names));exists=false(size(names));
for i=1:numel(names),exists(i)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{i}));if exists(i),saved{i}=evalin('base',names{i});end;end
model=evalin('base','ch5Runtime.modelName');block=[model,'/Strict CLF-CBF-QP Filter'];oldFunction=get_param(block,'FunctionName');
cleanup=onCleanup(@()ch5ForceRestoreV57(names,exists,saved,model,block,oldFunction)); %#ok<NASGU>
tasks=load(fullfile(root,'chapter5','artifacts','tasks','train.mat'),'tasks');task=tasks.tasks{4};
report=struct('pid',feature('getpid'),'completed',false,'rows',{{}},'formal',false,'error','');
try
    for mode=["ideal-force","nonideal-force"]
        r=ch5ReducedPrepareV48(4,mode);r.task=task;r.method="baseline";r.seed=0;r.c.episode_seconds=15;r.speed.stop_on_unsafe=false;
        [drift,G]=r.ad.dynamics(task.q0,task.qd0);r.initialForce=full(G)\(-full(drift));
        stewart=evalin('base','stewart');stewart=initializeStewartPose(stewart,'AP',task.q0(1:3)-r.physical.qHome(1:3),'ARB',ch5Rotation(task.q0(4:6)));
        if mode=="nonideal-force"
            actuator=makeNonidealForceActuator(r.physical,struct('forceRateLimit',r.c.actuator_slew_rate,'forceLagTimeConstant',r.c.actuator_time_constant, ...
                'forceInputDelay',r.c.actuator_delay,'forceDeadzone',ones(6,1)*r.c.actuator_deadzone,'initialForce',r.initialForce));
            stewart.nonidealForceActuator=actuator;assignin('base','nonidealForceActuator',actuator);
        end
        ch5ReleaseEpisodeData(r.mode,'baseline',r.task);assignin('base','stewart',stewart);assignin('base','ch5Runtime',r);
        set_param(block,'FunctionName','ch5TeacherV64Sfun','Parameters','ch5Runtime');timer=tic;set_param(model,'SimulationCommand','update');update=toc(timer);
        timer=tic;output=sim(model,'SimulationMode','normal','StopTime','15','ReturnWorkspaceOutputs','on','CaptureErrors','on');wall=toc(timer);
        records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
        file=fullfile(directory,sprintf('train_004_%s.mat',strrep(mode,'-','_')));save(file,'records','predictions','plans','output','r','-v7');
        metrics=ch5Metrics(records,output,r);audit=getappdata(0,'ch5Audit');row=struct('task',task.id,'mode',char(mode),'metrics',metrics,'wall_seconds',wall,'update_seconds',update,'simulation_error',output.ErrorMessage,'admitted',metrics.safe_success,'formal',false);
        save(file,'row','audit','-append');report.rows{end+1}=row;writeJSON(strrep(file,'.mat','.json'),row);writeJSON(fullfile(directory,'summary.json'),report);
        fprintf('V64 teacher%s safe%d arrival%d fallback%d acc_violation%.6f wall%.1fs\n',mode,metrics.all_constraints_valid,metrics.arrival,metrics.fallback_count,metrics.violations.acceleration.maximum,wall);
    end
    report.completed=numel(report.rows)==2;writeJSON(fullfile(directory,'summary.json'),report);
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');writeJSON(fullfile(directory,'summary.json'),report);rethrow(ex);end
end
function writeJSON(file,value)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(value));fclose(fid);
end
