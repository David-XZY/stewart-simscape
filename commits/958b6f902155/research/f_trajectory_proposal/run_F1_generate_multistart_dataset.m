function dataset=run_F1_generate_multistart_dataset(tasks,c)
% Formal dataset generation; invoke only after experiment compute is configured.
rows={}; attempts=struct('taskId',{},'start',{},'passed',{},'reason',{});
implementationHash=abfdHash(struct('planner',abfdPlannerImplementationHash(), ...
    'generator',abfdFileHash([mfilename('fullpath'),'.m'])));
guarded=isfield(c,'plannerProfile') && any(strcmp(c.plannerProfile,{'guarded-exact-v1','guarded-adaptive-v2'}));
setupCasadiIpoptMa27(fileparts(fileparts(fileparts(mfilename('fullpath')))));
for i=1:numel(tasks)
    task=tasks(i); provider=buildDynamicsProvider(task.model);
    disc=buildTwoPhaseIHSDiscretization(task.scene,task.scene.phase.numIntervalsApproach,task.scene.phase.numIntervalsInsertion);
    for j=1:c.candidateCount
        file=fullfile(c.outputDir,task.taskId,sprintf('start_%02d.mat',j));
        hashConfig=rmfield(c,'outputDir');
        inputHash=abfdHash(struct('task',task,'config',hashConfig,'start',j, ...
            'providerHash',provider.hash,'implementationHash',implementationHash));
        timer=tic;
        try
            if isfile(file)
                saved=load(file); assert(strcmp(saved.inputHash,inputHash), ...
                    'ABFD:CheckpointMismatch','Dataset checkpoint does not match task.'); result=saved.result;
            else
                if guarded
                    [cp,seedReport]=buildObstacleAwareSplineSeed(task,j);
                    assert(seedReport.passed,'ABFD:GeometricSeedRejected', ...
                        'Geometric start %d rejected: %s, violation=%g.',j,seedReport.status,seedReport.maxViolation);
                else
                    cp=zeros(6,10); cp(:,1:3)=repmat(task.scene.q0,1,3); cp(:,8:10)=repmat(task.scene.qWaypoint,1,3);
                    for n=4:7, cp(:,n)=(1-(n-3)/5)*task.scene.q0+(n-3)/5*task.scene.qWaypoint; end
                    stream=RandStream('mt19937ar','Seed',c.seed+task.seed+j);
                    if j>1, cp(:,4:7)=cp(:,4:7)+[.10*randn(stream,3,4);.10*randn(stream,3,4)]; end
                    seedReport=struct('method','unoptimized random spline','start',j);
                end
                [~,initial]=buildLearnedInitialGuessIHSID(cp,task.model,task.scene,disc,provider);
                local=c; local.designMode='sequential';
                local.outputDir=fullfile(c.outputDir,task.taskId,sprintf('start_%02d_details',j));
                result=solveAbfdIhsid(task.model,task.scene,provider,local,initial);
                result.proposalSeed=seedReport; result.candidateTotalTime=toc(timer);
                if ~isfolder(fileparts(file)), mkdir(fileparts(file)); end
                save(file,'result','inputHash');
            end
            reason=result.solverStatus;
            if isfield(result,'failureReason'), reason=result.failureReason; end
            attempts(end+1)=struct('taskId',task.taskId,'start',j,'passed',result.passed,'reason',reason); %#ok<AGROW>
            if result.passed
                s=fitTrajectorySpline(result.bundle.refs.t,result.bundle.refs.q,task.scene);
                rows{end+1}=struct('x',task.context(:).','y',s.features(:).', ...
                    'group',task.taskId,'split',task.split,'objective',result.objective, ...
                    'mode',routeSignature(result.bundle.refs,task.scene),'start',j, ...
                    'seconds',result.candidateTotalTime,'solveSeconds',result.solveTime, ...
                    'iterations',result.iterations,'retries',result.retryCount); %#ok<AGROW>
            end
        catch exception
            if strcmp(exception.identifier,'ABFD:CheckpointMismatch'), rethrow(exception); end
            % Retain terminal failure checkpoints, including rejected seeds.
            % A different implementation/configuration requires a new run directory.
            if ~isfile(file)
                result=struct('passed',false,'solverStatus',exception.identifier, ...
                    'failureReason',exception.message,'candidateTotalTime',toc(timer));
                if ~isfolder(fileparts(file)), mkdir(fileparts(file)); end
                save(file,'result','inputHash');
            end
            attempts(end+1)=struct('taskId',task.taskId,'start',j,'passed',false,'reason',exception.message); %#ok<AGROW>
        end
        writeAbfdJson(fullfile(c.outputDir,'proposal_attempts.json'),attempts);
    end
end
assert(~isempty(rows),'ABFD:EmptyDataset','No densely feasible trajectories were generated.');
data=[rows{:}]; dataset=struct('schemaVersion',1,'kind','trajectory-proposal-dataset', ...
    'x',vertcat(data.x),'y',vertcat(data.y),'groups',{{data.group}},'split',{{data.split}}, ...
    'objective',[data.objective],'routeMode',{{data.mode}},'implementationHash',implementationHash, ...
    'startIndex',[data.start],'candidateSeconds',[data.seconds],'solveSeconds',[data.solveSeconds], ...
    'iterations',[data.iterations],'retryCount',[data.retries]);
writeAbfdJson(fullfile(c.outputDir,'proposal_dataset.json'),dataset);
end
function signature=routeSignature(refs,scene)
% Geometric route classes, not a proof of topological homotopy equivalence.
signature='base';
for j=4:numel(scene.hood.obstacles)
    ids=find(refs.t<=scene.phase.durationApproach);
    delta=refs.q(1:3,ids)-scene.hood.obstacles(j).center_S;
    [~,k]=min(sum(delta.^2,1)); side=sign(scene.box.R_S(:,2).'*delta(:,k));
    signature=[signature,sprintf('_%d',side)]; %#ok<AGROW>
end
end
