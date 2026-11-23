function c=bdConfig(runName)
% Isolated implementation; legacy chapter3 numerical files remain unchanged.
if nargin<1, runName='boundary_dev_v1'; end
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'chapter3')); c=c3Setup(runName);
c.version='boundary_impl_v1'; c.denseStep=.005; c.qualityStep=.005;
c.controlCount=16; c.referenceBudget=120; c.wallBudget=120;
c.envelope=struct('scope','declared_idealized_not_CAD_verified', ...
    'platformRadius',.55,'platformHalfHeight',.05,'legRadius',.06, ...
    'supportSmoothingSquared',1e-6,'baseRadius',.80,'baseCenter',[0;0;-.10],'baseHalfHeight',.10);
c.positionCap=inf; c.rotationCap=inf; c.qualityFrozen=false;
c.batchTotals=[1,4,8,16,32,64]; c.stageFractions=[.5,.25,.25];
c.retimeControls=12; c.retimeIterations=40; c.pathGrid=257;
c.lengthError=1e-8; c.timeError=1e-6; c.returnPolicy='physical';
c.trainingAllowed=false; c.formalAllowed=false;
files=[dir(fullfile(fileparts(mfilename('fullpath')),'*.m'));dir(fullfile(fileparts(mfilename('fullpath')),'*.py'))];
sources=cell(numel(files),2);
for k=1:numel(files), sources{k,1}=files(k).name; sources{k,2}=dpHash(fileread(fullfile(files(k).folder,files(k).name))); end
c.codeHash=dpHash(sources);
provenance=struct('version',c.version,'codeHash',c.codeHash,'sources',{sources},'configuration',c,'matlab',version);
file=fullfile(c.output,['provenance_',c.codeHash,'.json']);
if ~isfile(file), dpWriteJson(file,provenance); end
end
