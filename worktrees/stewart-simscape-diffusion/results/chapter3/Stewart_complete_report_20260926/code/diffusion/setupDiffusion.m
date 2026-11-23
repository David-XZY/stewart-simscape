function c=setupDiffusion(overrides)
% Independent full-horizon planner. MATLAB R2019a or later.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'diffusion'),fullfile(root,'opt_minimal','core'), ...
    fullfile(root,'opt_minimal','ihsid'),fullfile(root,'src'));
c=struct('schemaVersion',1,'root',root,'output',fullfile(root,'results','diffusion'), ...
    'intervals',60,'candidateCount',8,'seed',20260915,'maxIter',2000, ...
    'maxCpuSeconds',600,'denseStep',.01,'nearStep',.001,'clearance',.01, ...
    'guidanceStep',.05,'guidanceWeights',[10,1,1,.1,.01], ...
    'printLevel',0,'dependencyRoot','');
if nargin>0
    names=fieldnames(overrides);
    for k=1:numel(names)
        assert(isfield(c,names{k}),'DP:Config','Unknown configuration field.');
        c.(names{k})=overrides.(names{k});
    end
end
assert(c.clearance==.01 && c.intervals==60,'DP:Contract','v1 requires 10 mm and 60 intervals.');
dependency=getenv('STEWART_DEPENDENCY_ROOT');
if ~isempty(c.dependencyRoot), dependency=c.dependencyRoot; end
if isempty(dependency)
    dependency=root;
    if ~isfile(fullfile(root,'lib','casadi-windows-matlabR2018b-v3.7.2','casadiMEX.mexw64'))
        dependency=fullfile(fileparts(root),'stewart-simscape');
    end
end
persistent ready;
if isempty(ready) || ~strcmp(ready,dependency)
    setupCasadiIpoptMa27(dependency); ready=dependency;
end
if ~isfolder(c.output), mkdir(c.output); end
end
