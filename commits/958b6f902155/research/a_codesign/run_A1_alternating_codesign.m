function result=run_A1_alternating_codesign(model,scene,provider,c,initial,firstPrimalStart)
if nargin<5, initial=[]; end
if nargin<6, firstPrimalStart=[]; end
assert(any(strcmp(c.controllerMode,{'fullstate-tvlqr','geometric-D'})), ...
    'ABFD:CodesignController','A requires its full-state TVLQR or geometric-D controller.');
base=c; base.designMode='sequential';
initialTube=checkAbfdInitialTube(model,scene,c);
if ~initialTube.lengthPassed
    if ~isfolder(c.outputDir), mkdir(c.outputDir); end
    save(fullfile(c.outputDir,'codesign_initial_rejection.mat'),'initial','initialTube','c');
    error('ABFD:InitialTubeInfeasible','Fixed-start tightened leg-length margin is %.9g m.',initialTube.minimumLengthMargin);
end
if isstruct(initial) && isfield(initial,'bundle') && isfield(initial,'z')
    result=revalidateAbfdPlanningSeed(initial,model,scene,provider,c);
else
    result=solveAbfdIhsid(model,scene,provider,base,initial);
end
assert(result.passed,'ABFD:NoFeasibleSeed','Sequential seed failed dense acceptance.');
physicalSeed=result;
if ~isfolder(c.outputDir), mkdir(c.outputDir); end
save(fullfile(c.outputDir,'codesign_seed.mat'),'physicalSeed','initialTube','c');
history=struct('iteration',{},'accepted',{},'cost',{},'radius',{},'reason',{},'maxIneq',{});
feasibility=evaluateAbfdCodesignFeasibility(result.bundle,provider,c);
merit=result.objective+feasibility.trackingCost;
initialFeasibility=feasibility;
writeAbfdJson(fullfile(c.outputDir,'codesign_initial_feasibility.json'),initialFeasibility);
result.passed=feasibility.passed;
for it=1:c.maxOuterIterations
    trial=[];
    design=buildAbfdSchedule(result.bundle.refs,provider,c);
    [allTimes,order]=sort([result.traj.t,result.traj.tc]);
    allQ=[result.traj.Xnode(1:6,:),result.traj.Xmid(1:6,:)];
    design.nominal=struct('q',allQ(:,order),'t',allTimes);
    try
        local=c; local.outputDir=fullfile(c.outputDir,sprintf('outer_%02d',it));
        primalStart=[]; if it==1, primalStart=firstPrimalStart; end
        trial=solveAbfdIhsid(model,scene,provider,local,result.initial,result.z,design,primalStart);
        next=inf; trialFeasibility=struct('passed',false,'maxIneq',inf);
        if trial.passed
            trialFeasibility=evaluateAbfdCodesignFeasibility(trial.bundle,provider,c);
            % NLP objective already includes tracking; compare a fresh common merit.
            frozen=base; nominalNlp=struct('provider',provider,'config',frozen,'center',[], 'design',[],'buildOnly',true);
            nlp=buildCasadiTrackableIHSNLP(model,scene,trial.disc,trial.initial,struct(),nominalNlp);
            val=nlp.eval(trial.z); next=full(val)+trialFeasibility.trackingCost;
        end
        [radius,accept]=updateCodesignTrustRegion(c.trustRadius,merit,next,trial.passed, ...
            feasibility.maxIneq,trialFeasibility.maxIneq,c.constraintTolerance);
        reason=trial.solverStatus;
    catch exception
        next=inf; accept=false; radius=max(1e-4,c.trustRadius/2); reason=exception.message;
        trialFeasibility=struct('passed',false,'maxIneq',inf);
    end
    history(end+1)=struct('iteration',it,'accepted',accept,'cost',next,'radius',radius,'reason',reason, ...
        'maxIneq',trialFeasibility.maxIneq); %#ok<AGROW>
    delta=abs(merit-next)/max(1,abs(merit));
    if accept, result=trial; merit=next; feasibility=trialFeasibility; end
    c.trustRadius=radius;
    result.passed=feasibility.passed && any([history.accepted]);
    result.codesign=struct('initialFeasibility',initialFeasibility,'finalFeasibility',feasibility, ...
        'acceptedIterations',sum([history.accepted]),'pairPassed',feasibility.passed,'passed',result.passed);
    if ~isfolder(c.outputDir), mkdir(c.outputDir); end
    save(fullfile(c.outputDir,'codesign_checkpoint.mat'),'result','trial','history','feasibility','physicalSeed','c');
    if isstruct(trial) && isfield(trial,'solverStatus') && strcmp(trial.solverStatus,'Maximum_CpuTime_Exceeded') && trial.iterations==0
        result.stoppingReason='Derivative evaluation exhausted the CPU budget before an iteration; radius-only retries cannot provide a refinement.';
        break;
    end
    if accept && feasibility.passed && delta<c.convergenceTolerance, break; end
end
result.history=history; result.finalMerit=merit;
result.design=buildAbfdSchedule(result.bundle.refs,provider,c);
result=finalizeAbfdCodesignAcceptance(result,initialFeasibility,feasibility,history,physicalSeed.bundle);
save(fullfile(c.outputDir,'codesign_result.mat'),'result','c');
end
