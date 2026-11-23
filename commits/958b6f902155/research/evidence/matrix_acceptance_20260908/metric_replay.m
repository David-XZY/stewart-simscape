try
 cd('D:/NPU/Matlab_code/stewart-simscape-abfd');addpath('research');setupAbfdResearch();
 out='results/abfd/compute/matrix_acceptance_check_20260908';
 files={'integration/assessAbfdExperimentMetrics.m','integration/buildAbfdExperimentMatrix.m','integration/run_abfd_matrix.m','integration/run_abfd_full_experiments.m','integration/generateAbfdReport.m','integration/runAbfdSimscapeCase.m','integration/runAbfdExperiment.m','tests/testExperimentAcceptance.m'};
 checks=cellfun(@(f)struct('file',f,'messages',checkcode(fullfile('research',f),'-id')),files,'UniformOutput',false);
 writeAbfdJson(fullfile(out,'checkcode.json'),checks); assert(all(cellfun(@(v)isempty(v.messages),checks)),'ABFD:Checkcode','Static checks failed.');
 testExperimentAcceptance();
 original='results/abfd/compute/B_frozen_holdout_20260908/execution'; folders=dir(fullfile(original,'task_*')); rows=struct('taskId',{},'passed',{},'originalPassed',{},'manifestHash',{});
 for k=1:numel(folders)
  directory=fullfile(folders(k).folder,folders(k).name); m=readAbfdJsonUtf8(fullfile(directory,'experiment','manifest.json')); previous=readAbfdJsonUtf8(fullfile(directory,'status.json'));
  acceptance=assessAbfdExperimentMetrics(m.details.metrics,m.config); assert(acceptance.passed==previous.passed,'ABFD:HoldoutAcceptance','Frozen per-task acceptance changed.');
  rows(k)=struct('taskId',folders(k).name,'passed',acceptance.passed,'originalPassed',previous.passed,'manifestHash',abfdFileHash(fullfile(directory,'experiment','manifest.json')));
  target=fullfile(out,'report_inputs',folders(k).name);mkdir(target);copyfile(fullfile(directory,'experiment','manifest.json'),fullfile(target,'manifest.json'));
 end
 assert(numel(rows)==12 && sum([rows.passed])==4,'ABFD:HoldoutDenominator','All frozen tasks must remain.');
 report=generateAbfdReport(fullfile(out,'report_inputs'));assert(report.completedCount==12 && report.passedCount==4 && report.rejectedCount==8,'ABFD:ReportAcceptance','Summary lost acceptance failures.');
 writeAbfdJson(fullfile(out,'status.json'),struct('state','complete','passed',true,'tests','testExperimentAcceptance','frozenHoldoutAudit',rows,'reportCompleted',report.completedCount,'reportPassed',report.passedCount,'reportRejected',report.rejectedCount,'scope','Configuration and metric regression; read-only replay of all twelve real frozen B+D results; no new simulation or training'));
catch e
 writeAbfdJson('results/abfd/compute/matrix_acceptance_check_20260908/status.json',struct('state','failed','identifier',e.identifier,'message',getReport(e,'extended','hyperlinks','off')));rethrow(e);
end