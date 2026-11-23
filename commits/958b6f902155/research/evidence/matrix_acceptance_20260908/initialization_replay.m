try
 cd('D:/NPU/Matlab_code/stewart-simscape-abfd');addpath('research');setupAbfdResearch();out='results/abfd/compute/matrix_common_initialization_check_v2_20260908';
 files={'common/makeAbfdConfig.m','integration/applyAbfdStaticPreload.m','integration/buildAbfdProviderForTask.m','integration/abfdRunFingerprint.m','integration/buildAbfdExperimentMatrix.m','integration/run_abfd_matrix.m','integration/run_abfd_full_experiments.m','integration/runAbfdExperiment.m','integration/runAbfdSimscapeCase.m','integration/generateAbfdReport.m','integration/assessAbfdExperimentMetrics.m','tests/testExperimentAcceptance.m','tests/testCommonPlantInitialization.m'};
 checks=cellfun(@(f)struct('file',f,'messages',checkcode(fullfile('research',f),'-id')),files,'UniformOutput',false);writeAbfdJson(fullfile(out,'checkcode.json'),checks);assert(all(cellfun(@(v)isempty(v.messages),checks)),'ABFD:Checkcode','Static check failed.');
 testExperimentAcceptance(); report=testCommonPlantInitialization('results/abfd/compute/B_frozen_holdout_20260908/execution');assert(numel(report.rows)==12);report.state='complete';writeAbfdJson(fullfile(out,'status.json'),report);
catch e
 writeAbfdJson('results/abfd/compute/matrix_common_initialization_check_v2_20260908/status.json',struct('state','failed','identifier',e.identifier,'message',getReport(e,'extended','hyperlinks','off')));rethrow(e);
end