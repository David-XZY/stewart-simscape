function summary=bdProbe()
% Independent geometric development probe. No learned model is loaded.
% A valid output is a physical witness, not a successful IHSID reference label.
c=bdConfig('boundary_geometric_probe_v1'); task=bdMakeTask([],[0,0,0,7.5],c);
base=bdBaseline(task,16); u=linspace(0,1,10); wave=sin(pi*(u+.05)/1.1).^2;
pool={}; ranks=[]; p=sgPhysics(task.model); m=task.model; counter=0;
for yaw=[-50,-35,-20,20,35,50]
    for lift=[0,.04,.08,.12]
        for shift=[-.08,0,.08]
            counter=counter+1; cp=base; cp(6,4:13)=cp(6,4:13)+deg2rad(yaw)*wave;
            cp(3,4:13)=cp(3,4:13)+lift*wave; cp(2,4:13)=cp(2,4:13)+shift*wave;
            b=struct('kind','spline','cp',cp,'T',task.T); tr=bdReconstruct(b,linspace(0,task.T,81));
            [L,Ld,Ldd,F]=p.point(tr.q,tr.v,tr.a); gaps=sgClearanceBatch(tr.q,task); envelope=bdRobotEnvelope(task,tr.q,c);
            violation=[max(0,m.lmin-full(L))/.1;max(0,full(L)-m.lmax)/.1;max(0,abs(full(Ld))./m.actuator.ldotMax-1);max(0,abs(full(Ldd))./m.actuator.lddotMax-1);max(0,abs(full(F))/2000-1);max(0,.01-gaps)/.01;max(0,.01-envelope.gaps)/.01];
            ranks(counter)=sum(violation(:).^2); pool{counter}=struct('bundle',b,'yawDegrees',yaw,'liftM',lift,'shiftM',shift,'coarseRank',ranks(counter));
        end
    end
end
[~,order]=sort(ranks); records={}; success={};
for id=order(1:12)
    candidate=pool{id}; [ok,r]=bdAccept(candidate.bundle,task,'physical',tic,30,c);
    record=rmfield(candidate,'bundle'); record.id=id; record.passed=ok; record.reason=r.reason; records{end+1}=record;
    if ok
        success{end+1}=candidate; bundle=candidate.bundle;
        save(fullfile(c.output,sprintf('feasible_%03d.mat',id)),'task','bundle','record','-v7');
        bdExportTrajectory(bundle,task,fullfile(c.output,sprintf('feasible_%03d.json',id)),c);
    end
end
summary=struct('candidateCount',counter,'denseChecks',12,'qualified',numel(success),'records',{records}, ...
    'trainingCalls',0,'diffusionCalls',0,'ihsidCalls',0,'mechanismEstablished',false, ...
    'scope','Independent geometric probe; witnesses do not establish coupled necessity or a successful IHSID reference.');
save(fullfile(c.output,'probe.mat'),'task','pool','summary','-v7'); dpWriteJson(fullfile(c.output,'summary.json'),summary);
fprintf('BD_PROBE qualified=%d / %d dense checks\n',numel(success),12);
end
