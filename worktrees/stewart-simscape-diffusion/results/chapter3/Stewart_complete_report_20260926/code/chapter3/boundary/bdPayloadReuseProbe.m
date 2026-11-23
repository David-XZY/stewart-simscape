function summary=bdPayloadReuseProbe()
% Reuse the 12 previously accepted geometric probes, not a trained library.
c=bdConfig('boundary_payload_reuse_v1');
assert(~isfile(fullfile(c.output,'summary.json')),'BD:ImmutableRun','Use a new version for reruns.');
folder=fullfile(c.root,'results','chapter3','boundary_geometric_probe_v1');
files=dir(fullfile(folder,'feasible_*.mat')); cases=[100,1.3,.15,7.5;400,1.2,.15,2;400,1.3,.15,2];
dpWriteJson(fullfile(c.output,'manifest.json'),struct('cases',cases,'sourceFiles',{{files.name}}, ...
    'codeHash',c.codeHash,'scope','Previously accepted geometric probes; not IHSID training history.'));
rows={};
for k=1:size(cases,1)
    v=cases(k,:); base=bdMakeTask([],[0,0,0,v(4)],c); task=bdPayloadTask(base,v(1),v(2),v(3),c);
    for j=1:numel(files)
        z=load(fullfile(folder,files(j).name)); bundle=z.bundle; bundle.T=task.T;
        [ok,r]=bdAccept(bundle,task,'physical',tic,60,c);
        row=struct('caseId',k,'parameters',v,'source',files(j).name,'passed',ok,'reason',r.reason);
        if ok
            row.quality=bdQuality(bundle,task,c);
            save(fullfile(c.output,sprintf('case_%d_source_%d.mat',k,j)),'task','bundle','row','-v7');
        end
        rows{end+1}=row;
    end
end
% A single fixed-path retiming attempt for the mass-only dynamic failure.
v=cases(2,:); base=bdMakeTask([],[0,0,0,v(4)],c); task=bdPayloadTask(base,v(1),v(2),v(3),c);
z=load(fullfile(folder,'feasible_050.mat')); path=z.bundle;
clock=tic; [timed,log]=bdRetime(path,task,clock,10,c);
[ok,r]=bdAccept(timed,task,'physical',tic,60,c);
retime=struct('parameters',v,'source','feasible_050.mat','solve',log,'physicalPassed',ok,'reason',r.reason, ...
    'note','Offline diagnostic with independent acceptance after solver budget. Not an online 10 s return claim.');
if ok, retime.quality=bdQuality(timed,task,c); end
save(fullfile(c.output,'retime.mat'),'task','timed','log','r','-v7');
summary=struct('rows',{rows},'retiming',retime,'trainingCalls',0,'diffusionCalls',0,'ihsidCalls',0, ...
    'retimingCalls',1,'scope','Limited previously generated geometry reuse, not full history diagnosis.');
dpWriteJson(fullfile(c.output,'summary.json'),summary);
for k=1:3, fprintf('PAYLOAD_REUSE case=%d pass=%d/%d\n',k,sum(cellfun(@(r)r.caseId==k&&r.passed,rows)),numel(files)); end
fprintf('PAYLOAD_RETIME physical=%d status=%s\n',ok,log.status);
end
