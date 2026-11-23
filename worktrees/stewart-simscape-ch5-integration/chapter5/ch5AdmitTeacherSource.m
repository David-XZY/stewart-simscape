function yes=ch5AdmitTeacherSource(result,file,expected)
% Only enumerated, byte-hashed stationary-obstacle teacher episodes may
% retain earlier prechecks/CV expressions. Moving-obstacle episodes from the
% aliasing implementation are excluded. No development/formal exceptions.
yes=false;[c,root]=ch5Init();registry=fullfile(root,'chapter5','teacher_source_admissions.json');
if ~isfield(result,'source_identity'),return;end
if strcmp(result.method,'diffusion')
    if ~isfield(result.source_identity,'diffusion_source_sha256')|| ...
            ~strcmp(result.source_identity.diffusion_source_sha256,expected.diffusion_source_sha256),return;end
end
if strcmp(result.source_identity.baseline_source_sha256,expected.baseline_source_sha256),yes=true;return;end
if ~strcmp(result.method,'baseline')||~isfield(result.task,'split')||~ismember(result.task.split,{'train','validation'})||~isfile(registry),return;end
admission=jsondecode(fileread(registry));if ~strcmp(admission.version,c.version),return;end
if ~strcmp(expected.files.ch5Ihsid_m,admission.new_ihsid_sha256),return;end
if result.task.family==4||any(result.task.motion_velocity~=0,'all'),return;end
parents=admission.parents;parentIndex=find(strcmp({parents.baseline_source_sha256},result.source_identity.baseline_source_sha256),1);
if isempty(parentIndex),return;end
parent=parents(parentIndex);
names=fieldnames(expected.files);
for k=1:numel(names)
    name=names{k};if strcmp(name,'ch5Ihsid_m'),continue;end
    if ~isfield(result.source_identity.files,name)||~strcmp(result.source_identity.files.(name),expected.files.(name)),return;end
end
if ~strcmp(result.source_identity.files.ch5Ihsid_m,parent.ihsid_sha256),return;end
[~,name,ext]=fileparts(file);entries=admission.runs;index=find(strcmp({entries.file},[name,ext]),1);
if isempty(index),return;end
yes=strcmp(entries(index).baseline_source_sha256,result.source_identity.baseline_source_sha256)&& ...
    strcmp(hashFile(file),entries(index).sha256)&&strcmp(hashFile(fullfile(root,'chapter5',parent.source_file)),parent.ihsid_sha256);
end
function value=hashFile(file)
digest=java.security.MessageDigest.getInstance('SHA-256');fid=fopen(file,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
while ~feof(fid),digest.update(fread(fid,1048576,'*uint8'));end
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
