function manifest=ch5VerifyTestTasks()
[c,root]=ch5Init();base=fullfile(root,'chapter5','artifacts');file=fullfile(base,'tasks','test.mat');
manifestFile=[file,'.manifest.json'];assert(isfile(manifestFile),'ch5:TaskManifestMissing','Formal task draw has no completed identity manifest.');
manifest=jsondecode(fileread(manifestFile));
assert(strcmp(manifest.version,c.version)&&manifest.tasks==120);
assert(strcmp(manifest.task_sha256,hashFile(file)),'ch5:TaskDrawChanged','Frozen formal task draw changed.');
assert(strcmp(manifest.freeze_sha256,hashFile(fullfile(base,'freeze.json'))),'ch5:TaskFreezeMismatch','Formal task draw belongs to another freeze.');
end
function value=hashFile(file)
fid=fopen(file,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),digest.update(fread(fid,1048576,'*uint8'));end
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
