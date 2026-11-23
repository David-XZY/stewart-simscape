function c=c3Setup(runName)
% Separate experiment: legacy numerical sources and results are immutable.
if nargin<1, runName='chapter3_v3'; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'diffusion'),fullfile(root,'study'),fullfile(root,'chapter3'));
c=setupDiffusion(); c.source=fullfile(root,'results','diffusion','formal_v1');
if ~isfile(fullfile(c.source,'dataset.json')), c.source=fullfile(root,'results','remote','diffusion','formal_v1'); end
c.output=fullfile(root,'results','chapter3',runName);
if ~isfolder(c.output), mkdir(c.output); end
c.modelFile=fullfile(c.output,'training','model.json'); c.datasetFile=fullfile(c.output,'dataset.json');
c.qualityStep=.005; c.wallBudget=600; c.maxCpuSeconds=120;
c.repairBudget=10; c.repairIterations=40; c.repairCandidates=2;
c.batchTotals=[1,4,8,16,32,64]; c.solverStarts=8; c.seed=2026091707;
c.weightNames={'balanced','smooth','economy'};
c.weights=[.25,.25,.25,.25;.20,.25,.35,.20;.35,.20,.15,.30];
c.metricNames={'positionLengthM','rotationLengthRad','positionAccelRms','angularAccelRms', ...
    'legAccelRms','positionJerkRms','angularJerkRms','legJerkRms','forceRmsN','forceRateRms'};
c.groups={1:2,3:5,6:8,9:10}; c.metricFloor=1e-12*ones(1,10);
c.detourFloors=[.01,pi/180]; c.qualityMultiplier=1.5;
end
