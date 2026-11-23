function summary=bdPayloadPilot()
% Bounded model-sensitivity diagnostic. Zero optimization/training/model calls.
c=bdConfig('boundary_payload_pilot_v1');
summaryFile=fullfile(c.output,'summary.json');
assert(~isfile(summaryFile),'BD:ImmutableRun','Pilot already exists; use a new version for a rerun.');
source=fullfile(c.root,'results','chapter3','boundary_geometric_probe_v1','feasible_050.mat');
z=load(source); bundle=z.bundle; base=bdMakeTask([],[0,0,0,7.5],c);
nominal=bdPayloadTask(base,100,1.2,.15,c);
assert(norm(nominal.model.dynamics.inertiaAtCOM_P-base.model.dynamics.inertiaAtCOM_P,'fro')<1e-10);
assert(norm(nominal.model.dynamics.comP-base.model.dynamics.comP)<1e-12);
assert(numel(nominal.context)==96&&isequal(nominal.context(1:93),base.context));
changed=bdPayloadTask(base,200,1.2,.15,c);
assert(~strcmp(bdProblemKey(nominal,c),bdProblemKey(changed,c)));
tr=bdReconstruct(bundle,[0,base.T/2,base.T]);
p0=sgPhysics(nominal.model); p1=sgPhysics(changed.model);
[L0,~,~,F0,J0]=p0.point(tr.q,tr.v,tr.a); [L1,~,~,F1,J1]=p1.point(tr.q,tr.v,tr.a);
assert(max(abs(full(L0(:)-L1(:))))<1e-12&&max(abs(full(J0(:)-J1(:))))<1e-12);
assert(max(abs(full(F0(:)-F1(:))))>1);
assert(isequal(sgClearanceBatch(tr.q,nominal),sgClearanceBatch(tr.q,changed)));
checks={'nominal_inertia_and_COM_reproduction','96D_condition_contract','mass_cache_invalidation', ...
    'mass_changes_force_not_kinematics_or_collision'};
% Manifest frozen before path acceptance; invalid endpoints stay in results.
specs={};
for mass=[50,100,200,400]
    for lengthM=[.8,1.2,1.3]
        for radiusM=[.10,.15]
            for duration=[7.5,2]
                specs{end+1}=struct('mass',mass,'length',lengthM,'radius',radiusM,'T',duration);
            end
        end
    end
end
manifest=struct('schema','payload_diagnostic_v1','tasks',{specs},'sourcePathHash',bdFileHash(source), ...
    'codeHash',c.codeHash,'fixedFixtureOffsets',[0,0],'fixedMount',[0,0,.15], ...
    'noEccentricityParameter',true,'independentTaskCount',numel(specs),'formalTrainingCalls',0,'optimizerCalls',0);
dpWriteJson(fullfile(c.output,'manifest.json'),manifest); save(fullfile(c.output,'source_path.mat'),'bundle','-v7');
rows={};
for k=1:numel(specs)
    spec=specs{k}; started=tic;
    base=bdMakeTask([],[0,0,0,spec.T],c);
    task=bdPayloadTask(base,spec.mass,spec.length,spec.radius,c); b=bundle; b.T=spec.T;
    t=linspace(0,task.T,ceil(task.T/c.denseStep)+1); tr=bdReconstruct(b,t);
    gap=sgClearanceBatch(tr.q,task); [minimum,index]=min(gap(:)); [obstacle,sample]=ind2sub(size(gap),index);
    witness=[]; if minimum<.01, witness=bdDistanceWitness(tr.q(:,sample),task,obstacle); end
    [accepted,r]=bdAccept(b,task,'physical',tic,60,c); quality=bdQuality(b,task,c);
    pp=sgPhysics(task.model); [~,~,~,staticF]=pp.point([task.q0,task.qGoal],zeros(6,2),zeros(6,2));
    row=struct('taskId',task.taskId,'taskHash',task.hash,'parameters',spec, ...
        'endpointPassed',task.endpointPassed,'endpointReason',task.endpointReason, ...
        'passed',accepted&&task.endpointPassed,'reason',r.reason,'rawPathPassed',accepted, ...
        'minimumSeparationLower',minimum,'gapViolationWitness',witness,'staticForcePeakN',max(abs(full(staticF(:)))), ...
        'quality',quality,'totalMass',task.model.dynamics.totalMass, ...
        'COM',task.model.dynamics.comP,'inertiaCOM',task.model.dynamics.inertiaAtCOM_P,'seconds',toc(started));
    if ~task.endpointPassed, row.reason='boundary_rejected'; end
    rows{end+1}=row; dpWriteJson(fullfile(c.output,sprintf('task_%03d.json',k)),row);
    fprintf('PAYLOAD %d/%d m=%g L=%g r=%g T=%g endpoint=%d pass=%d reason=%s F=%.1f\n',k,numel(specs),spec.mass,spec.length,spec.radius,spec.T,task.endpointPassed,row.passed,row.reason,quality.forcePeakN);
end
summary=struct('rows',{rows},'checks',{checks},'count',numel(rows),'passed',sum(cellfun(@(x)x.passed,rows)), ...
    'endpointRejected',sum(cellfun(@(x)~x.endpointPassed,rows)),'optimizerCalls',0,'trainingCalls',0,'diffusionCalls',0, ...
    'scope','One fixed geometric path under uniform time scaling; this is not a library exclusion or diffusion advantage result.', ...
    'conditionSchema','96D proposed cylinder family; production training and retrieval pipelines not migrated.');
dpWriteJson(summaryFile,summary);
fprintf('PAYLOAD_DONE passed=%d/%d endpointRejected=%d\n',summary.passed,summary.count,summary.endpointRejected);
end
