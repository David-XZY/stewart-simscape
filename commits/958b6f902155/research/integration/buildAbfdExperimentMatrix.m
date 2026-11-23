function matrix=buildAbfdExperimentMatrix(base,caseOverrides)
% Common settings remain common; model-specific observations are explicit.
if nargin<2, caseOverrides=struct(); end
ids={'E0','EA','EB','EF','ED','EAD','EABD','EABFD','EA_SEQUENTIAL_TVLQR', ...
    'D_FIXED_LQI','D_SCHEDULED_LQI','D0','D_FIXED_GEOMETRIC'};
assert(isstruct(caseOverrides) && isscalar(caseOverrides) && ...
    all(ismember(fieldnames(caseOverrides),ids)),'ABFD:MatrixCaseOverrides','Unknown experiment override.');
scoped={'staticBalanceFile','inertiaCalibrationFile','temporalFile','allowPilotTemporal'};
assert(~any(isfield(base,scoped)),'ABFD:MatrixScopedOption', ...
    'Put B-specific startup and temporal settings in explicit EB/EABD/EABFD caseOverrides, not the shared base.');
locked={'modelMode','plannerMode','controllerMode','designMode','dVariant','outputDir', ...
    'plantProfile','plantOverrides','stateSource','sampleTime','feedbackBudget','Q','RF', ...
    'initialCovariance','processSpectralDensity','seed','attitudePerturbation', ...
    'qpEnabled','dobEnabled','antiWindupEnabled','governorEnabled','trajectoryFile', ...
    'constraintTolerance','dynamicsTolerance','onlineP95Limit','allowTestFixtures','plantInitializationFile'};
matrix=struct('id',{},'config',{});
for k=1:numel(ids)
    id=ids{k}; c=base;
    c.modelMode='analytic'; c.plannerMode='ihsid'; c.controllerMode='baseline-lqi'; c.designMode='sequential';
    if any(strcmp(id,{'EB','EABD','EABFD'})), c.modelMode='residual-B'; end
    if any(strcmp(id,{'EF','EABFD'})), c.plannerMode='learned-warmstart-F'; end
    if any(strcmp(id,{'EA','EAD','EABD','EABFD'})), c.designMode='codesign-A'; end
    if any(strcmp(id,{'EA','EA_SEQUENTIAL_TVLQR'})), c.controllerMode='fullstate-tvlqr'; end
    if any(strcmp(id,{'ED','EAD','EABD','EABFD','D0','D_FIXED_GEOMETRIC'})), c.controllerMode='geometric-D'; c.dVariant='tvlqr'; end
    if strcmp(id,'D0'), c.dVariant='d0'; end
    if strcmp(id,'D_FIXED_GEOMETRIC'), c.dVariant='fixed'; end
    if strcmp(id,'D_SCHEDULED_LQI'), c.controllerMode='scheduled-lqi'; end
    if isfield(caseOverrides,id)
        changes=caseOverrides.(id);
        assert(isstruct(changes) && isscalar(changes),'ABFD:MatrixCaseOverrides','Each override must be a scalar struct.');
        names=fieldnames(changes);
        assert(~any(ismember(names,locked)),'ABFD:MatrixComparisonChanged', ...
            'Case overrides cannot change algorithms, reference selection, common object, state source, costs, budget or acceptance.');
        for j=1:numel(names), c.(names{j})=changes.(names{j}); end
    end
    c.outputDir=fullfile(base.outputDir,id);
    try
        c=makeAbfdConfig(c);
    catch e
        error('ABFD:MatrixCaseConfig','Invalid %s configuration: %s',id,e.message);
    end
    matrix(end+1)=struct('id',id,'config',c); %#ok<AGROW>
end
end
