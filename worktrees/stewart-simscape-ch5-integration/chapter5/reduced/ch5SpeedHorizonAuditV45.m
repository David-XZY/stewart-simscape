function report=ch5SpeedHorizonAuditV45()
[~,root]=ch5Init();base=fullfile(root,'chapter5','artifacts','reduced_probe');directory=fullfile(base,'horizon_audit_v45');
assert(~isfolder(directory),'ch5:ExistingAudit','Retain the previous revision.');mkdir(directory);
report=struct('pid',feature('getpid'),'identity',ch5ReducedIdentity(),'rows',{{}},'accepted',false,'error','');
try
    for mode=["nonideal-force","ideal-force"]
        r=ch5ReducedPrepareV45(4,mode);r.reduced=struct('velocity_weight',1,'factor_derivatives',false);k=ch5ReducedKernelsV45(r);
        x=[r.task.q0;r.task.qd0];if r.plant.n>12,x=[x;repmat(r.initialForce,3,1)];end
        for variant=1:3
            U=repmat(r.initialForce,1,r.c.H)+(variant-1)*2*sin((1:6)'*(1:r.c.H)/11);
            [nodes,values,trace]=k.preview(x,U);
            [oldN,oldV,oldT]=r.plant.rollout(x,[U,repmat(U(:,end),1,30)]);count=numel(r.plant.traceTime)*r.c.H;
            errors=[norm(full(nodes)-full(oldN(:,1:r.c.H+1)),inf),norm(full(values)-full(oldV(:,1:count)),inf),norm(full(trace)-full(oldT(:,1:count)),inf)];
            report.rows{end+1}=struct('mode',char(mode),'variant',variant,'preview_errors',errors,'accepted',max(errors)<1e-10);
        end
        fullRuntime=r;fullRuntime.c.H=60;fullRuntime.c.version=[ch5ConfigVersion(root),'-speed-reduced-v1'];oldK=ch5ReducedKernelsV4(fullRuntime);
        [goal,o]=ch5Observe(r.task,0);setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',6443));
        [~,~,oldC]=ch5ReducedPriorCandidatesV31(x,r.initialForce,goal,o,fullRuntime,oldK);
        setappdata(0,'ch5Random',RandStream('mt19937ar','Seed',6443));[~,~,newC]=ch5ReducedPriorV43(x,r.initialForce,goal,o,r,k);
        error=0;for j=1:numel(oldC),error=max(error,norm(newC{j}.U-oldC{j}.U(:,1:30),inf));end
        report.rows{end+1}=struct('mode',char(mode),'variant','prior_prefix','prefix_error',error,'accepted',error<1e-10);
    end
    files={'ch5ReducedPrepareV45.m','ch5ReducedPriorV43.m','ch5ReducedControlV45.m','ch5ReducedV45Sfun.m','ch5ReducedKernelsV45.m','ch5ReducedPreviewV43.m'};
    for j=1:numel(files)
        issues=checkcode(fullfile(root,'chapter5','reduced',files{j}),'-id');assert(~any(strcmp({issues.id},'SYNER')),'ch5:SyntaxError','New horizon source did not parse.');
    end
    report.accepted=all(cellfun(@(v)v.accepted,report.rows));
catch ex,report.error=getReport(ex,'extended','hyperlinks','off');write();rethrow(ex);end
write();assert(report.accepted,'ch5:HorizonMismatch','Short-horizon physics or action decoding changed.');
    function write()
        save(fullfile(directory,'audit.mat'),'report','-v7');
        fid=fopen(fullfile(directory,'summary.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
    end
end
function version=ch5ConfigVersion(root)
config=jsondecode(fileread(fullfile(root,'chapter5','config.json')));version=config.version;
end
