function result=ch5RunEpisode(task,mode,method,seed,c,file)
wall=tic;identity=ch5SourceIdentity();result=struct('task',task,'mode',string(mode),'method',string(method),'seed',seed,'version',c.version,'status','initializing','source_identity',identity);
if isfile(file)
    previous=load(file,'result');
    if strcmp(previous.result.version,c.version)&&ismember(previous.result.status,{'completed','simulation_failed'})
        if isfield(previous.result,'source_identity')
            assert(ch5AdmitTeacherSource(previous.result,file,identity), ...
                'ch5:DevelopmentSourceChanged','A completed episode comes from changed shared/control code; preserve it and use a new experiment directory/version.');
        end
        result=previous.result;writeResultJson(file,result);return;
    end
    error('ch5:ExistingVersion','An existing incompatible or setup-failed episode must be audited before retry.');
end
lock=acquireLock(file);lockCleanup=onCleanup(@()releaseLock(lock));
% Another worker can finish this episode while this worker waits for its lock.
if isfile(file)
    previous=load(file,'result');
    assert(strcmp(previous.result.version,c.version)&&ismember(previous.result.status,{'completed','simulation_failed'}));
    assert(~isfield(previous.result,'source_identity')||ch5AdmitTeacherSource(previous.result,file,identity));
    result=previous.result;writeResultJson(file,result);return;
end
attemptId=char(java.util.UUID.randomUUID());
result.attempt_id=attemptId;result.journal_directory=fullfile([file,'.journal'],['attempt_',attemptId]);
if isfield(task,'split')&&strcmp(task.split,'test')
    freezeFile=fullfile(fileparts(fileparts(file)),'freeze.json');
    result.freeze_manifest_sha256=fileHash(freezeFile);
    taskManifest=ch5VerifyTestTasks();result.test_task_sha256=taskManifest.task_sha256;
end
appendAttempt(file,struct('attempt_id',attemptId,'status','started','pid',feature('getpid'), ...
    'utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ss.SSSXXX')), ...
    'version',c.version,'source_identity',identity,'task',task,'mode',mode,'method',method,'seed',seed,'journal_directory',result.journal_directory));
setappdata(0,'ch5Records',{});setappdata(0,'ch5Predictions',{});setappdata(0,'ch5Plans',{});setappdata(0,'ch5Audit',[]);output=[];
try
    initialization=tic;runtime=ch5PrepareSimscape(task,mode,method,seed,c);
    runtime.journalDirectory=result.journal_directory;if ~isfolder(runtime.journalDirectory),mkdir(runtime.journalDirectory);end
    assignin('base','ch5Runtime',runtime);
    if string(method)=="diffusion",result.model_sha256=fileHash(runtime.weights_file);end
    result.initialization_seconds=toc(initialization);result.status='running';
    output=sim(runtime.modelName,'SimulationMode','normal','StopTime',num2str(c.episode_seconds), 'ReturnWorkspaceOutputs','on','CaptureErrors','on');
    records=getappdata(0,'ch5Records');predictions=getappdata(0,'ch5Predictions');plans=getappdata(0,'ch5Plans');
    if ~isempty(output.ErrorMessage)
        result.status='simulation_failed';result.error=struct('identifier','Simulink:CapturedError','message',output.ErrorMessage);
        if isempty(records)||contains(output.ErrorMessage,'MATLAB S-Function')||contains(output.ErrorMessage,'MATLAB S-函数')
            result.status='setup_failed';
        end
        audit=[];
    else
        result.metrics=ch5Metrics(records,output,runtime);audit=getappdata(0,'ch5Audit');result.status='completed';
    end
catch ex
    result.error=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
    % An uncaught controller, export or metric exception is an unresolved
    % implementation error, not an algorithm outcome to fill the run count.
    result.status='setup_failed';
    % Preserve a Simscape output even if its subsequent metric audit fails.
    records={};predictions={};plans={};audit=[];
    if isappdata(0,'ch5Records'),records=getappdata(0,'ch5Records');end
    if isappdata(0,'ch5Predictions'),predictions=getappdata(0,'ch5Predictions');end
    if isappdata(0,'ch5Plans'),plans=getappdata(0,'ch5Plans');end
    if isappdata(0,'ch5Audit'),audit=getappdata(0,'ch5Audit');end
end
result.wall_seconds=toc(wall);result.attempt_id=attemptId;
pending=[file,'.pending'];save(pending,'result','records','predictions','plans','output','audit','-v7');
movefile(pending,file);writeResultJson(file,result);
appendAttempt(file,struct('attempt_id',attemptId,'status',result.status,'wall_seconds',result.wall_seconds));
fprintf('%s %s %s seed=%d status=%s wall=%.3fs\n',task.id,mode,method,seed,result.status,result.wall_seconds);
end
function lock=acquireLock(file)
path=[file,'.lock'];handle=java.io.RandomAccessFile(path,'rw');channel=handle.getChannel();
while true
    held=channel.tryLock();if ~isempty(held),break;end
    pause(2);
end
lock=struct('handle',handle,'channel',channel,'held',held);
if handle.length()>0&&~isfile(file)
    handle.seek(0);old=char(handle.readLine());
    if ~isempty(old),appendAttempt(file,struct('status','interrupted','prior_owner',old,'reason','Prior owner released the OS lock without a final result'));end
end
% Keep the lock inode: unlinking it would allow a second independent lock.
handle.setLength(0);handle.writeBytes(jsonencode(struct('pid',feature('getpid'), ...
    'created',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')))));
end
function releaseLock(lock)
lock.held.release();lock.channel.close();lock.handle.close();
end
function appendAttempt(file,value)
fid=fopen([file,'.attempts.jsonl'],'a','n','UTF-8');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value));fclose(fid);
end
function writeResultJson(file,result)
[directory,name]=fileparts(file);target=fullfile(directory,[name,'.json']);pending=[target,'.pending'];
fid=fopen(pending,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(result));fclose(fid);movefile(pending,target,'f');
end
function value=fileHash(file)
digest=java.security.MessageDigest.getInstance('SHA-256');fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));
while ~feof(fid),digest.update(fread(fid,1048576,'*uint8'));end
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
