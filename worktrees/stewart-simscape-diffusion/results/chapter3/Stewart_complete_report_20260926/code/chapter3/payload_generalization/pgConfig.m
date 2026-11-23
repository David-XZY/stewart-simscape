function c=pgConfig(runName)
if nargin<1, runName='payload_generalization_v1'; end
here=fileparts(mfilename('fullpath')); addpath(fullfile(fileparts(here),'boundary'));
c=bdConfig(runName); c.version='payload_generalization_v1';
c.contextSchema='pg_basic99_v1'; c.signatureSchema='pg_signature354_v1';
c.geometryVersion='pg_convex_support_v1'; c.physicsVersion='pg_uniform_rigid_v1';
c.trainingSeeds=[20260919,20260920,20260921]; c.samplingSeeds=c.trainingSeeds;
c.onlineBudgets=[.5,1,2,5]; c.primaryBudget=2; c.referenceBudget=120;
c.developmentMaxAttempts=576; c.pilotMaxAttempts=32;
c.referenceResourceHours=72; c.libraryDiagnosticHours=24;
c.noiseSchedule=pgNoiseSchedule(100); c.noiseScheduleHash=dpHash(c.noiseSchedule);
c.methods={'ihsid','retrieval','ddpm_basic','ddpm_signature','ddpm_physloss','ddpm_guided','regression'};
c.guidanceEta=.03;
c.trainingAllowed=false; c.formalAllowed=false;
f=[dir(fullfile(here,'*.m'));dir(fullfile(here,'*.py'));dir(fullfile(here,'protocol.json'))];
f=f(~startsWith({f.name},'report_'));
s=cell(numel(f),2); for k=1:numel(f), s{k,1}=f(k).name; s{k,2}=dpHash(fileread(fullfile(here,f(k).name))); end
c.codeHash=dpHash(s); c.numericalSources=s;
end
