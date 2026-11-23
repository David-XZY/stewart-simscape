try
 root='D:/XieZhenyu/stewart-abfd-runs/symmetric-9a39db5';out='D:/XieZhenyu/stewart-abfd-runs/symmetric-9a39db5/results/abfd/compute/A_symmetric_covariance_20260908';cd(root);addpath('research');setupAbfdResearch();setupCasadiIpoptMa27(root);
 files={'research/a_codesign/abfdPlannerImplementationHash.m','research/a_codesign/buildAbfdCovarianceShootingSeed.m','research/a_codesign/buildAbfdSampledTrackingExpressions.m','research/a_codesign/convertAbfdSampledCovarianceCoordinates.m','research/a_codesign/solveAbfdIhsid.m','research/common/makeAbfdConfig.m','research/remote/run_abfd_combined_chain_probe.m','research/remote/run_abfd_combined_chain_solve.m','research/remote/run_abfd_combined_reduction_probe.m','research/tests/run_abfd_light_tests.m','research/tests/testCovarianceShooting.m','research/tests/testGuardedCodesignSolver.m'}; checks=struct('file',{},'hash',{},'messages',{});
 for k=1:numel(files),file=fullfile(root,files{k});checks(k)=struct('file',files{k},'hash',abfdFileHash(file),'messages',checkcode(file,'-id'));end
 writeAbfdJson(fullfile(out,'checkcode.json'),checks);
 assert(all(arrayfun(@(item)isempty(item.messages),checks)),'ABFD:StaticCheck','Static inspection produced findings.');
 rejected=false;try,makeAbfdConfig(struct('covarianceCoordinates','symmetric'));catch e,rejected=strcmp(e.identifier,'ABFD:CovarianceCoordinatesConfig');end
 assert(rejected,'ABFD:CovarianceCoordinateConfigTest','Coordinates without shooting must be rejected.');
 ordinary=testCovarianceShooting();writeAbfdJson(fullfile(out,'ordinary_covariance.json'),ordinary);
 symmetric=testCovarianceShooting([],'symmetric');writeAbfdJson(fullfile(out,'symmetric_covariance.json'),symmetric);
 reduced=testReducedHsNlp([],true,'local-midpoint');writeAbfdJson(fullfile(out,'reduced_regression.json'),reduced);
 solver=testGuardedCodesignSolver(fullfile(out,'fixture_solver'),'covariance-shooting-hs-reduced','limited-memory',0,'','','local-midpoint','symmetric');
 writeAbfdJson(fullfile(out,'solver_contract.json'),solver);
 writeAbfdJson(fullfile(out,'phase_status.json'),struct('state','running','stage','full-original-value-audit','shortChecksPassed',true));
 oracle='D:/XieZhenyu/stewart-abfd-runs/f-holdout-aa3f100/results/abfd/compute/A_kkt_export_20260908/diagnostic';
 previous='D:/XieZhenyu/stewart-abfd-runs/exact-recovery-f3a07ce/results/abfd/compute/A_exact20ms_recovered_20260908/optimization/outer_01/guard_01';
 checkpoint=fullfile(previous,'solver_segments','segment_003.mat');
 values=run_abfd_combined_reduction_probe(oracle,fullfile(previous,'attempt.mat'),checkpoint,fullfile(out,'values'),'local-midpoint',false,'symmetric');
 writeAbfdJson(fullfile(out,'batch_status.json'),struct('state','complete','shortChecksPassed',true,'values',values, ...
   'scope','Short value/derivative/refusal contracts and complete original coordinate equivalence; full derivative and optimization acceptance remain separate'));
catch e
 if ~exist('out','var'),out='D:/XieZhenyu/stewart-abfd-runs/symmetric-9a39db5/results/abfd/compute/A_symmetric_covariance_20260908';end
 writeAbfdJson(fullfile(out,'failure.json'),struct('state','failed','identifier',e.identifier,'message',getReport(e,'extended','hyperlinks','off')));rethrow(e);
end
