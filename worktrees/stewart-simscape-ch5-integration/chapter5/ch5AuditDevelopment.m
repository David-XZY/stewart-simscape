function report=ch5AuditDevelopment()
% Inspect complete development records before drawing any holdout tasks.
% Algorithm infeasibility/solver iteration limits remain valid outcomes;
% uncaught projection/postcheck exceptions require repair before freezing.
[c,root]=ch5Init();base=fullfile(root,'chapter5','artifacts');directory=fullfile(base,'development');
files=dir(fullfile(directory,'*_force_*.mat'));assert(numel(files)==c.development_tasks*12);
expected=ch5SourceIdentity();report=struct('version',c.version,'runs',numel(files), ...
    'diffusion_source_sha256',expected.diffusion_source_sha256,'projection_exceptions',0, ...
    'postcheck_exceptions',0,'diffusion_control_calls',0,'fallbacks',0,'raw_files',[]);
raw=cell(numel(files),1);exceptions={};
for k=1:numel(files)
    file=fullfile(files(k).folder,files(k).name);sample=load(file,'result','predictions','records');result=sample.result;
    assert(strcmp(result.version,c.version)&&strcmp(result.task.split,'development'));
    assert(ismember(result.status,{'completed','simulation_failed'}));
    assert(ch5AdmitTeacherSource(result,file,expected),'ch5:DevelopmentSourceMismatch','Development records use changed control code.');
    if strcmp(result.method,'diffusion')
        model=fullfile(base,'models',sprintf('seed_%d.mat',result.seed));
        assert(strcmp(hashFile(model),result.model_sha256),'ch5:DevelopmentModelChanged','Development model has changed.');
        for j=1:numel(sample.predictions)
            entry=sample.predictions{j};prediction=entry.prediction;attempts=prediction.denoise_attempts;
            assert(isequal(size(attempts),[c.ddim_steps,c.candidates]));
            assert(numel(prediction.final_attempts)==c.candidates&&numel(prediction.checks)==c.candidates);
            bad=cellfun(@(a)strcmp(a.status,'exception'),attempts);
            finalBad=cellfun(@(a)strcmp(a.status,'exception'),prediction.final_attempts);
            postBad=cellfun(@(a)isfield(a,'postcheck_error'),prediction.checks);
            report.projection_exceptions=report.projection_exceptions+sum(bad,'all')+sum(finalBad);
            report.postcheck_exceptions=report.postcheck_exceptions+sum(postBad);
            if any(bad,'all')||any(finalBad)||any(postBad)
                exceptions{end+1}=struct('file',files(k).name,'time',entry.time, ...
                    'projection',{attempts(bad)},'final',{prediction.final_attempts(finalBad)},'postcheck',{prediction.checks(postBad)});
            end
            chosen=prediction.chosen;report.diffusion_control_calls=report.diffusion_control_calls+1;
            report.fallbacks=report.fallbacks+(chosen==0);
            if chosen>0
                assert(prediction.checks{chosen}.accepted);
                candidate=prediction.candidates{chosen};n=12;if strcmp(result.mode,'nonideal-force'),n=30;end
                assert(isequal(size(candidate.X),[n,c.H+1])&&isequal(size(candidate.U),[6,c.H]));
                record=sample.records{j};assert(abs(record.time-entry.time)<1e-9);
                assert(norm(candidate.X(:,1)-record.x,inf)<1e-9);
                assert(norm(diff([record.previous_u,candidate.U],1,2)-candidate.delta,inf)<1e-9);
                assert(norm(record.u-candidate.U(:,1),inf)<1e-9);
            end
        end
    end
    raw{k}=struct('file',files(k).name,'sha256',hashFile(file),'status',result.status);
end
report.raw_files=vertcat(raw{:});report.passed=report.projection_exceptions==0&&report.postcheck_exceptions==0;
save(fullfile(base,'validation','development_audit.mat'),'report','exceptions','-v7');
assert(report.passed,'ch5:DevelopmentExceptions','Development contains projection/postcheck exceptions; repair them before freezing.');
end
function value=hashFile(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),digest.update(fread(fid,1048576,'*uint8'));end
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
