function report=run_B1_identify_elasticity(inertiaRoot,replayRoot,tasksFile,outputDir)
% Identify passive terms on train groups only. Initial pose is observable
% context, not a fitted test-load parameter or an extra neural-network input.
setupAbfdResearch();
assert(~isfile(fullfile(outputDir,'status.json')),'ABFD:ElasticityRunExists','Use a fresh output directory.');
if ~isfolder(outputDir), mkdir(outputDir); end
parentFile=fullfile(inertiaRoot,'residual_dataset.json'); parent=jsondecode(fileread(parentFile));
accepted=jsondecode(fileread(fullfile(inertiaRoot,'relabeling_verification.json')));
assert(accepted.passed && strcmp(accepted.datasetHash,abfdFileHash(parentFile)), ...
    'ABFD:ElasticityParent','The independently verified inertia dataset is required.');
tasks=jsondecode(fileread(tasksFile)); measured=load(fullfile(inertiaRoot,'responses_checkpoint.mat'),'measurements');
measurements=measured.measurements; groups={measurements.taskId};
assert(all(strcmp({measurements.split},'train')) && ...
    isequal(sort(string(groups(:))),sort(string(parent.physical.trainingGroups(:)))), ...
    'ABFD:TrainingLeakage','Elasticity identification must use exactly the frozen inertia training groups.');
report=struct('state','running','passed',false,'parentDatasetHash',abfdFileHash(parentFile), ...
    'tasksHash',abfdFileHash(tasksFile),'completed',0,'expected',numel(measurements));
writeAbfdJson(fullfile(outputDir,'status.json'),report);
try
    design=[]; target=[];
    for k=1:numel(measurements)
        physical=measurements(k); task=tasks(strcmp({tasks.taskId},physical.taskId));
        assert(isscalar(task) && strcmp(task.split,'train'),'ABFD:TrainingLeakage','Unknown training task.');
        calibration=fullfile(inertiaRoot,physical.taskId,'status.json');
        assert(strcmp(abfdFileHash(calibration),physical.calibrationHash),'ABFD:ElasticitySource','Training response changed.');
        index=find(strcmp({parent.pairedSourceCases.taskId},physical.taskId));
        file=fullfile(replayRoot,physical.taskId,'result.mat');
        assert(isscalar(index) && strcmp(abfdFileHash(file),parent.pairedSourceCases(index).resultHash), ...
            'ABFD:ElasticitySource','Training replay changed.');
        saved=load(file,'result'); log=saved.result.log; m=task.model; local=m;
        local.dynamics.totalMass=physical.mass; local.dynamics.comP=physical.com;
        local.dynamics.inertiaAtCOM_P=physical.inertia;
        rest=sgpIK(log.q(:,1),m); n=numel(log.t); Ys=zeros(6,n); Yd=Ys; residual=Ys;
        data=buildResidualWrenchTarget(log,m);
        for j=1:n
            q=log.q(:,j); v=log.qd(:,j); kin=sgpIK(q,m); J=sgpJacobian(q,m);
            Ys(:,j)=J.Jv.'*(kin.L-rest.L); Yd(:,j)=J.Jv.'*(J.Jq*v);
            residual(:,j)=data.wrench(j,:).'-computeCompositeRequiredWrench(q,v,log.qdd(:,j),local);
        end
        % Within-task centering removes constant offsets before coefficient
        % identification. It does not subtract offsets from exported labels.
        D=diag([ones(1,3),ones(1,3)/m.rB]);
        Ys=D*(Ys-mean(Ys,2)); Yd=D*(Yd-mean(Yd,2)); residual=D*(residual-mean(residual,2));
        design=[design;[Ys(:),Yd(:)]/sqrt(n)]; target=[target;residual(:)/sqrt(n)]; %#ok<AGROW>
        report.completed=k; writeAbfdJson(fullfile(outputDir,'status.json'),report);
    end
    assert(rank(design)==2 && all(isfinite([design(:);target])),'ABFD:ElasticityRank','Passive coefficients are not identifiable.');
    [coefficient,resnorm,~,exitFlag]=lsqnonneg(design,target);
    assert(exitFlag>0,'ABFD:ElasticityFit','Nonnegative passive fit did not converge.');
    elastic=struct('reference','initial-leg-length-v1','stiffness',coefficient(1),'damping',coefficient(2), ...
        'trainingGroups',{groups},'responseHashes',{{measurements.calibrationHash}}, ...
        'identification','Within-task centered, equal-task weighted, nonnegative force/velocity regression', ...
        'resnorm',resnorm,'units','N/m and N*s/m','sourceHash',abfdFileHash([mfilename('fullpath'),'.m']));
    output=parent; output.physical.elasticity=elastic; output.y=zeros(size(parent.y));
    contexts=struct([]); cursor=0;
    for k=1:numel(parent.pairedSourceCases)
        source=parent.pairedSourceCases(k); task=tasks(strcmp({tasks.taskId},source.taskId));
        file=fullfile(replayRoot,source.taskId,'result.mat');
        assert(strcmp(abfdFileHash(file),source.resultHash),'ABFD:ElasticitySource','Frozen replay changed.');
        saved=load(file,'result'); log=saved.result.log; n=numel(log.t); ids=cursor+(1:n);
        assert(isequal([log.q;log.qd].',parent.x(ids,:)),'ABFD:ElasticityRows','Frozen state rows differ.');
        rest=sgpIK(log.q(:,1),task.model);
        for j=1:n
            q=log.q(:,j); J=sgpJacobian(q,task.model); kin=sgpIK(q,task.model);
            passive=coefficient(1)*(kin.L-rest.L)+coefficient(2)*(J.Jq*log.qd(:,j));
            output.y(ids(j),:)=parent.y(ids(j),:)-(J.Jv.'*passive).';
        end
        row=struct('taskId',source.taskId,'split',source.split,'initialPose',log.q(:,1), ...
            'initialLength',rest.L,'firstRow',cursor+1,'rows',n,'sourceHash',source.resultHash);
        if isempty(contexts), contexts=row; else, contexts(k)=row; end
        cursor=cursor+n;
    end
    assert(cursor==size(parent.x,1) && all(isfinite(output.y(:))),'ABFD:ElasticityRows','Incomplete or invalid labels.');
    output.elasticityContexts=contexts; output.inertiaDatasetHash=report.parentDatasetHash;
    output.elasticityProtocol='Fit coefficients on train-only independent inertia responses and motion; initial pose observed per trajectory; no held-out parameter fit';
    output.targetDefinition='Actual world wrench minus train-identified rigid inertia and passive spring/damping at observed initial leg lengths';
    writeAbfdJson(fullfile(outputDir,'residual_dataset.json'),output);
    report.state='complete'; report.passed=true; report.elasticity=elastic; report.rows=cursor;
    report.datasetHash=abfdFileHash(fullfile(outputDir,'residual_dataset.json'));
    report.scope=output.elasticityProtocol; writeAbfdJson(fullfile(outputDir,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    writeAbfdJson(fullfile(outputDir,'status.json'),report); rethrow(e);
end
end
