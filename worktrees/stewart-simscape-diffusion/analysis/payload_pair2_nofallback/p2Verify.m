function p2Verify(requestFile)
r=jsondecode(fileread(requestFile)); c=pgConfig('payload_pair2_nofallback_v1/verification');
assert(strcmp(c.codeHash,r.originalCodeHash),'P2:Code','Original source identity differs.');
report=p2FlowTests();
% Actual CPU sampler: pair boundaries cannot alter candidate-ID streams.
model=jsondecode(fileread(fullfile(r.root,r.models(1).relativeFile)));
task=pgMakeTask(r.jobs(1).spec,c); asset=struct('model',model,'callContext',pgEncodeContext(task,'basic'));
clock=tic; [pair,~]=pgCandidates('ddpm_basic',task,[65,66],asset,clock,inf,c,20260919);
[one,~]=pgCandidates('ddpm_basic',task,65,asset,clock,inf,c,20260919);
[two,~]=pgCandidates('ddpm_basic',task,66,asset,clock,inf,c,20260919);
assert(isequal(pair{1}.bundle.cp,one{1}.bundle.cp)&&isequal(pair{2}.bundle.cp,two{1}.bundle.cp));
report.tests{end+1}='actual_sampler_pair_prefix_beyond_64';
% Same physical penalty and numerical steps as the frozen original repair.
c.returnPolicy='physical'; clock=tic; [old,~]=pgRepair(pair{1}.bundle,task,clock,inf,c);
clock=tic; [new,info]=p2Repair(pair{1}.bundle,task,clock,inf,c);
assert(max(abs(old.cp(:)-new.cp(:)))<1e-12&&info.iterations<=40);
report.tests{end+1}='repair_arithmetic_matches_original';
report.matlab=version; report.casadi=casadi.CasadiMeta.version(); report.threads=maxNumCompThreads;
assert(report.threads==1); dpWriteJson(fullfile(r.output,'verification.json'),report);
end
