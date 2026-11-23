function c=sgSetup(runName)
if nargin<1, runName='frozen_v1'; end
c=setupDiffusion(); addpath(fullfile(c.root,'study'));
c.source=fullfile(c.output,'formal_v1');
if ~isfile(fullfile(c.source,'training','model.json'))
    c.source=fullfile(c.root,'results','remote','diffusion','formal_v1');
end
c.output=fullfile(c.root,'results','study',runName);
c.modelFile=fullfile(c.source,'training','model.json');
c.datasetFile=fullfile(c.source,'dataset.json');
c.fastBudget=30; c.solverBudget=180; c.referenceBudget=600;
c.repairBudget=2; c.repairIterations=20; c.repairCandidates=2;
c.qualityStep=.005; c.maxCpuSeconds=180; c.seed=2026091601;
c.observationBudgets=[1,2,5,10,20,30]; c.candidateCounts=[1,4,8,16,32,64];
if ~isfolder(c.output), mkdir(c.output); end
end
