function ch5PrepareTasks(stage)
% The supervisor calls this once before launching episode workers.
[c,root]=ch5Init();[m,s]=ch5Physics(c);collision=ch5CollisionModel(m,s,c);
switch stage
    case 'teacher',splits=["train","validation"];
    case 'development',splits="development";
    case 'formal',ch5VerifyFreeze();splits="test";
    otherwise,error('ch5:Stage','Unknown task stage.');
end
for split=splits
    file=char(fullfile(root,'chapter5','artifacts','tasks',split+'.mat'));
    if isfile(file)
        saved=load(file,'tasks','version');assert(strcmp(saved.version,c.version));
        if split=="test"
            if ~isfile([file,'.manifest.json'])
                assert(isempty(dir(fullfile(root,'chapter5','artifacts','formal','*_force_*.json'))), ...
                    'ch5:TaskManifestMissing','Formal outcomes exist without a completed task draw manifest.');
                regenerated=ch5GenerateTasks(char(split),c,m,s,collision);
                assert(isequaln(saved.tasks,regenerated),'ch5:TaskDrawChanged','Interrupted formal draw does not reproduce frozen generation.');
                writeManifest(file,c,root,numel(saved.tasks));
            end
            ch5VerifyTestTasks();
        end
        continue;
    end
    tasks=ch5GenerateTasks(char(split),c,m,s,collision);version=c.version;
    pending=[file,'.pending'];save(pending,'tasks','version','-v7');movefile(pending,file);
    if split=="test",writeManifest(file,c,root,numel(tasks));ch5VerifyTestTasks();end
    fprintf('Generated %d %s tasks from physics-only checks.\n',numel(tasks),split);
end
end
function writeManifest(file,c,root,count)
manifest=struct('version',c.version,'tasks',count,'task_sha256',hashFile(file), ...
    'freeze_sha256',hashFile(fullfile(root,'chapter5','artifacts','freeze.json')),'generation_seed',c.test_task_seed);
target=[file,'.manifest.json'];fid=fopen([target,'.pending'],'w','n','UTF-8');assert(fid>=0);
fprintf(fid,'%s',jsonencode(manifest));fclose(fid);movefile([target,'.pending'],target);
end
function value=hashFile(file)
fid=fopen(file,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),digest.update(fread(fid,1048576,'*uint8'));end
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
