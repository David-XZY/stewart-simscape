function report=ch5SpeedReducedReportAudit()
% Export MATLAB string arrays natively; SciPy treats them as MCOS objects.
[~,root]=ch5Init();directory=fullfile(root,'chapter5','artifacts','reduced_probe','report_audit_v1');
assert(~isfile(fullfile(directory,'summary.json')),'ch5:ExistingAudit','Keep the existing report input audit.');
if ~isfolder(directory),mkdir(directory);end
saved=load(fullfile(root,'chapter5','artifacts','tasks','development.mat'),'tasks');
report=struct('source_identity',ch5SourceIdentity(),'tasks',{saved.tasks},'pid',feature('getpid'), ...
    'reduced_identity',ch5ReducedIdentity(),'task_file_sha256',digest(fullfile(root,'chapter5','artifacts','tasks','development.mat')));
fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end

function hash=digest(file)
fid=fopen(file,'rb');assert(fid>=0);bytes=fread(fid,inf,'*uint8');fclose(fid);
md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
