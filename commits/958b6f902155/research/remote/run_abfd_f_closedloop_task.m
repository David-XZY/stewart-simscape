function report=run_abfd_f_closedloop_task(inputDir,snapshotDir,taskId,outputDir)
% Execute all predefined physical/observer cases for the three frozen methods.
% Failed refinements stay failed; no new proposal, optimizer retry or fitting.
root=setupAbfdResearch(); setupCasadiIpoptMa27(root);
assert(~isfolder(outputDir),'ABFD:FClosedLoopExists','Use a fresh closed-loop directory.');
mkdir(outputDir); timer=tic;
writeAbfdJson(fullfile(outputDir,'status.json'),struct('state','running','stage','input-audit','taskId',taskId));
try
    delivery=loadAbfdFrozenFResult(inputDir,snapshotDir,taskId);
    task=delivery.task; cases=buildAbfdCaseMatrix(); p=buildDynamicsProvider(task.model);
    assert(numel(cases)==12 && numel(delivery.methods)==3,'ABFD:FClosedLoopScope', ...
        'All twelve physical/observer cases and all three methods are required.');
    sourceFiles={'research/integration/loadAbfdFrozenFResult.m','research/remote/run_abfd_f_closedloop_task.m', ...
        'research/integration/buildAbfdCaseMatrix.m','research/integration/runAbfdSimscapeCase.m', ...
        'research/integration/prepareAbfdSimscape.m','research/integration/makeAbfdRuntime.m', ...
        'research/integration/assessAbfdExperimentMetrics.m','research/integration/geometricTvlqrSimscapeSfun.m', ...
        'research/integration/stepAbfdIntervalObserver.m','research/integration/stepAbfdExternalObserver.m', ...
        'research/integration/stepAbfdController.m','research/integration/installAbfdSimscapeRuntime.m', ...
        'research/integration/readAbfdSimscapeLog.m'};
    hashes=cellfun(@(file)abfdFileHash(fullfile(root,file)),sourceFiles,'UniformOutput',false);
    frozen=struct('taskId',taskId,'taskHash',abfdHash(task),'cases',cases,'methods',{{delivery.methods.name}}, ...
        'inputFreezeHash',delivery.frozenFileHash,'transferManifestHash',delivery.transferManifestHash, ...
        'remotePlanningManifest',delivery.manifest,'sourceFiles',{sourceFiles},'sourceHashes',{hashes}, ...
        'scope','E0/EF execution of frozen ranked refinements, same nominal LQI definition and force budget; all physical/observer cases, no A/B claim, no training or replacement reference');
    writeAbfdJson(fullfile(outputDir,'execution_freeze.json'),frozen);
    records=struct('taskId',{},'caseId',{},'method',{},'status',{},'planningPassed',{},'passed',{},'seconds',{},'reason',{},'metrics',{});
    for j=1:numel(cases)
        for k=1:numel(delivery.methods)
            records(end+1)=struct('taskId',taskId,'caseId',cases(j).id,'method',delivery.methods(k).name, ...
                'status','pending','planningPassed',delivery.methods(k).planningPassed,'passed',false, ...
                'seconds',0,'reason','','metrics',struct()); %#ok<AGROW>
        end
    end
    publish('running');
    for j=1:numel(cases)
        for k=1:numel(delivery.methods)
            row=(j-1)*numel(delivery.methods)+k; method=delivery.methods(k); caseTimer=tic;
            c=delivery.planningConfig;
            c.modelMode='analytic'; c.controllerMode='baseline-lqi'; c.designMode='sequential';
            c.plannerMode='ihsid'; c.proposalFile=''; c.trajectoryFile='';
            if k>1
                c.plannerMode='learned-warmstart-F';
                c.proposalFile=fullfile(inputDir,'models',method.name,'proposal_model.json');
            end
            c.plantProfile=cases(j).profile; c.plantOverrides=cases(j).overrides; c.stateSource=cases(j).stateSource;
            c.outputDir=fullfile(outputDir,cases(j).id,method.name); c=makeAbfdConfig(c);
            assert(~any([c.qpEnabled,c.dobEnabled,c.antiWindupEnabled,c.governorEnabled]), ...
                'ABFD:FClosedLoopAblation','Main F comparison requires independent extra mechanisms disabled.');
            records(row).status='running'; publish('running');
            try
                if ~method.planningPassed
                    records(row).status='planning-rejected';
                    records(row).reason='Frozen refinement failed; retained without simulation, alternate reference or retry.';
                    exportExperimentManifest(c.outputDir,c,'failed',struct('stage','planning-rejected', ...
                        'taskId',taskId,'method',method.name,'caseId',cases(j).id,'provenance',method.provenance));
                else
                    profile=makeHighFidelityPlantProfile(c.plantProfile,c.plantOverrides);
                    bundle=method.bundle;
                    assert(bundle.publishPassed && strcmp(bundle.providerHash,p.hash),'ABFD:FClosedLoopReference');
                    bundle.origin=method.provenance;
                    if ~isfolder(c.outputDir), mkdir(c.outputDir); end
                    referenceFile=fullfile(c.outputDir,'reference_bundle.mat'); save(referenceFile,'bundle');
                    fingerprint=abfdHash(struct('executionFreeze',frozen,'config',c,'provenance',method.provenance, ...
                        'referenceHash',abfdFileHash(referenceFile)));
                    exportExperimentManifest(c.outputDir,c,'running',struct('taskId',taskId,'method',method.name, ...
                        'caseId',cases(j).id,'provenance',method.provenance,'fingerprint',fingerprint));
                    result=runAbfdSimscapeCase(bundle,p,c,profile); result.planning=method.planning;
                    result.provenance=method.provenance; result.referenceHash=abfdFileHash(referenceFile);
                    result.log.taskId=taskId; result.log.loadId=task.loadId;
                    result.log.trajectoryId=[taskId,'_',method.name];
                    file=fullfile(c.outputDir,'result.mat'); save(file,'result');
                    exportExperimentManifest(c.outputDir,c,'experiment-completed',struct('taskId',taskId, ...
                        'method',method.name,'caseId',cases(j).id,'provenance',method.provenance, ...
                        'metrics',result.metrics,'passed',result.passed,'acceptance',result.acceptance, ...
                        'fingerprint',fingerprint,'resultHash',abfdFileHash(file),'referenceHash',result.referenceHash));
                    records(row).status='experiment-completed'; records(row).passed=result.passed;
                    records(row).metrics=result.metrics; records(row).reason=strjoin(result.acceptance.reasons,', ');
                end
            catch e
                if bdIsLoaded('stewart_platform_model'), close_system('stewart_platform_model',0); end
                records(row).status='failed'; records(row).reason=[e.identifier,': ',e.message];
                exportExperimentManifest(c.outputDir,c,'failed',struct('taskId',taskId,'method',method.name, ...
                    'caseId',cases(j).id,'identifier',e.identifier,'message',e.message,'provenance',method.provenance));
            end
            records(row).seconds=toc(caseTimer); publish('running');
        end
    end
    generateAbfdReport(outputDir); publish('complete');
catch e
    failure=struct('state','failed','taskId',taskId, ...
        'identifier',e.identifier,'message',getReport(e,'extended','hyperlinks','off'));
    writeAbfdJson(fullfile(outputDir,'failure.json'),failure);
    if exist('records','var'), publish('failed'); else, writeAbfdJson(fullfile(outputDir,'status.json'),failure); end
    rethrow(e);
end
    function publish(state)
        report=struct('state',state,'taskId',taskId,'plannedCount',numel(records), ...
            'simulatedCount',sum(strcmp({records.status},'experiment-completed')), ...
            'planningRejectedCount',sum(strcmp({records.status},'planning-rejected')), ...
            'executionFailedCount',sum(strcmp({records.status},'failed')),'acceptedCount',sum([records.passed]), ...
            'seconds',toc(timer),'records',records, ...
            'scope','All declared E0/EF cases retained; execution completion does not imply acceptance or superiority; full ABFD is separate');
        writeAbfdJson(fullfile(outputDir,'status.json'),report);
    end
end
