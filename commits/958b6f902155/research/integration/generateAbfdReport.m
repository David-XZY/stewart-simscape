function report=generateAbfdReport(outputDir)
files=dir(fullfile(outputDir,'**','manifest.json'));
rows=struct('path',{},'status',{},'passed',{},'reportedPassed',{},'metrics',{},'acceptance',{},'reason',{});
for k=1:numel(files)
    file=fullfile(files(k).folder,files(k).name); m=readAbfdJsonUtf8(file);
    if ~all(isfield(m,{'status','config','details'})), continue; end
    metrics=[]; acceptance=struct(); passed=false; reportedPassed=false; reason='No completed metrics.';
    if isfield(m.details,'passed'), reportedPassed=m.details.passed; end
    if isfield(m.details,'metrics')
        metrics=m.details.metrics;
        try
            acceptance=assessAbfdExperimentMetrics(metrics,m.config);
            passed=strcmp(m.status,'experiment-completed') && acceptance.passed;
            reason=strjoin(acceptance.reasons,', ');
        catch e, reason=[e.identifier,': ',e.message];
        end
    end
    rows(end+1)=struct('path',file,'status',m.status,'passed',passed,'reportedPassed',reportedPassed, ...
        'metrics',metrics,'acceptance',acceptance,'reason',reason); %#ok<AGROW>
end
matrixFiles=dir(fullfile(outputDir,'**','matrix_status.json'));
planned=0; pending=0; running=0;
for k=1:numel(matrixFiles)
    records=readAbfdJsonUtf8(fullfile(matrixFiles(k).folder,matrixFiles(k).name));
    if isempty(records), continue; end
    planned=planned+numel(records); pending=pending+sum(strcmp({records.status},'pending'));
    running=running+sum(strcmp({records.status},'running'));
end
report=struct('runs',rows,'completedCount',sum(strcmp({rows.status},'experiment-completed')), ...
    'passedCount',sum([rows.passed]),'failedCount',sum(strcmp({rows.status},'failed')), ...
    'rejectedCount',sum(strcmp({rows.status},'experiment-completed') & ~[rows.passed]), ...
    'matrixPlannedCount',planned,'matrixPendingCount',pending,'matrixRunningCount',running, ...
    'status','Results summarize only existing manifests; missing runs are not accepted.');
writeAbfdJson(fullfile(outputDir,'abfd_report.json'),report);
fid=fopen(fullfile(outputDir,'abfd_report.md'),'w','n','UTF-8'); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# ABFD experiment evidence\n\nCompleted: %d; passed: %d; failed: %d.\n\n', ...
    report.completedCount,report.passedCount,report.failedCount);
fprintf(fid,'Matrix declared: %d; pending: %d; running: %d.\n\n',planned,pending,running);
fprintf(fid,'| Run | Status | Engineering, budget, P95 and applicable tube acceptance |\n|---|---|---|\n');
for k=1:numel(rows), fprintf(fid,'| %s | %s | %d |\n',rows(k).path,rows(k).status,rows(k).passed); end
fprintf(fid,'\nNo superiority claim is made from incomplete runs. Pareto comparisons require matched task, plant, state source and seed.\n');
end
