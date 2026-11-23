function report=run_B1_diagnose_conditioned_inertia(originalRoot,replayRoot,inertiaRoot,asset,outputDir)
% Training-only mechanism diagnosis from independent force/acceleration
% probes. No held-out response is read and no new control run is accepted.
setupAbfdResearch();
assert(~isfile(fullfile(outputDir,'status.json')),'ABFD:DiagnosisExists','Use a fresh diagnosis directory.');
if ~isfolder(outputDir), mkdir(outputDir); end
tasksFile=fullfile(originalRoot,'B_tasks_frozen.json'); tasks=jsondecode(fileread(tasksFile));
responseFile=fullfile(inertiaRoot,'responses_checkpoint.mat'); saved=load(responseFile,'measurements');
measurements=saved.measurements;
assert(all(strcmp({measurements.split},'train')) && numel(unique({measurements.taskId}))==numel(measurements), ...
    'ABFD:TrainingLeakage','Only distinct frozen training response probes may enter this diagnosis.');
parent=jsondecode(fileread(fullfile(inertiaRoot,'residual_dataset.json')));
report=struct('state','running','passed',false,'completed',0,'expected',numel(measurements), ...
    'assetHash',abfdFileHash(asset),'tasksHash',abfdFileHash(tasksFile), ...
    'responseFileHash',abfdFileHash(responseFile),'cases',[],'scope', ...
    'Counterfactual training-log wrench predictions using independent per-training-task inertia; no new closed loop or held-out acceptance');
writeAbfdJson(fullfile(outputDir,'status.json'),report); rows=struct([]);
try
    for k=1:numel(measurements)
        measured=measurements(k); id=measured.taskId; task=tasks(strcmp({tasks.taskId},id));
        assert(isscalar(task) && strcmp(task.split,'train'),'ABFD:TrainingLeakage','Task is not frozen training data.');
        replayFile=fullfile(replayRoot,id,'result.mat');
        source=parent.pairedSourceCases(strcmp({parent.pairedSourceCases.taskId},id));
        assert(isscalar(source) && strcmp(source.split,'train') && ...
            strcmp(source.resultHash,abfdFileHash(replayFile)),'ABFD:DiagnosisSource','Training replay changed.');
        sourceFile=fullfile(originalRoot,'B0',id,'result.mat'); original=load(sourceFile,'result');
        assert(strcmp(source.sourceHash,abfdFileHash(sourceFile)),'ABFD:DiagnosisSource','Original training source changed.');
        bundle=original.result.planning.bundle; profile=original.result.profile;
        calibrationFile=fullfile(inertiaRoot,id,'status.json');
        assert(strcmp(abfdFileHash(calibrationFile),measured.calibrationHash),'ABFD:DiagnosisSource','Independent training observation changed.');
        calibration=jsondecode(fileread(calibrationFile));
        base=buildDynamicsProvider(task.model,'residual-B',asset,false,calibration.pose(:));
        provider=conditionResidualMeanOnStaticBalance(base,bundle,profile,calibrationFile);
        local=task.model; local.dynamics.totalMass=measured.mass; local.dynamics.comP=measured.com;
        local.dynamics.inertiaAtCOM_P=measured.inertia;
        replay=load(replayFile,'result'); log=replay.result.log; data=buildResidualWrenchTarget(log,task.model);
        q0=calibration.pose(:); zeros6=zeros(6,1);
        shared0=computeCompositeRequiredWrench(q0,zeros6,zeros6,base.model);
        measured0=computeCompositeRequiredWrench(q0,zeros6,zeros6,local);
        initial=base.evaluate(q0,zeros6,zeros6); measuredW0=initial.Jv.'*calibration.equilibriumForce(:);
        physicalOffset=measuredW0-measured0; delta0=measured0-shared0;
        n=numel(log.t); errors=zeros(6,n,3); gravityCorrection=zeros(6,n); inertialCorrection=zeros(6,n);
        neuralVariation=zeros(6,n); net0=evaluateResidualMlp([q0;zeros6],base.network);
        for j=1:n
            q=log.q(:,j); v=log.qd(:,j); a=log.qdd(:,j);
            current=provider.evaluate(q,v,a);
            shared=computeCompositeRequiredWrench(q,v,a,base.model);
            actualRigid=computeCompositeRequiredWrench(q,v,a,local);
            gravityCorrection(:,j)=computeCompositeRequiredWrench(q,zeros6,zeros6,local)- ...
                computeCompositeRequiredWrench(q,zeros6,zeros6,base.model)-delta0;
            inertialCorrection(:,j)=actualRigid-shared-delta0-gravityCorrection(:,j);
            correction=gravityCorrection(:,j)+inertialCorrection(:,j);
            kin=sgpIK(q,task.model); J=sgpJacobian(q,task.model); e=base.elasticity;
            passive=J.Jv.'*(e.stiffness*(kin.L-e.referenceLength)+e.damping*(J.Jq*v));
            predictedPhysical=actualRigid+passive+physicalOffset;
            errors(:,j,1)=data.wrench(j,:).'-current.W;
            errors(:,j,2)=data.wrench(j,:).'-current.W-correction;
            errors(:,j,3)=data.wrench(j,:).'-predictedPhysical;
            neuralVariation(:,j)=evaluateResidualMlp([q;v],base.network)-net0;
        end
        D=diag([ones(1,3),ones(1,3)/task.model.rB]); rms=zeros(1,3); rmsComponents=zeros(6,3);
        for variant=1:3
            weighted=D*errors(:,:,variant); rms(variant)=sqrt(mean(weighted(:).^2));
            rmsComponents(:,variant)=sqrt(mean(errors(:,:,variant).^2,2));
        end
        row=struct('taskId',id,'split','train','rows',n,'measuredMass',measured.mass, ...
            'sharedMass',base.model.dynamics.totalMass,'wrenchRmsNEquivalent',rms, ...
            'componentRms',rmsComponents,'gravityCorrectionRms',sqrt(mean(gravityCorrection.^2,2)), ...
            'inertialCorrectionRms',sqrt(mean(inertialCorrection.^2,2)), ...
            'neuralVariationRms',sqrt(mean(neuralVariation.^2,2)), ...
            'calibrationHash',measured.calibrationHash,'replayHash',source.resultHash);
        if isempty(rows), rows=row; else, rows(k)=row; end
        save(fullfile(outputDir,[id,'.mat']),'row','errors','gravityCorrection','inertialCorrection','neuralVariation');
        report.completed=k; report.cases=rows; writeAbfdJson(fullfile(outputDir,'status.json'),report);
    end
    values=vertcat(rows.wrenchRmsNEquivalent);
    report.variants={'current static-conditioned shared-inertia B','same B plus measured inertia correction', ...
        'measured inertia and train-identified passive terms with independent static balance; no MLP mean'};
    report.equalTaskRmsNEquivalent=sqrt(mean(values.^2,1));
    report.medianTaskRmsNEquivalent=median(values,1);
    report.state='complete'; report.passed=true; writeAbfdJson(fullfile(outputDir,'status.json'),report);
catch e
    report.state='failed'; report.identifier=e.identifier; report.message=e.message;
    report.detail=getReport(e,'extended','hyperlinks','off'); writeAbfdJson(fullfile(outputDir,'status.json'),report); rethrow(e);
end
end
