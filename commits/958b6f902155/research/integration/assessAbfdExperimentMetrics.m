function acceptance=assessAbfdExperimentMetrics(metrics,c)
% Execution completion is separate from engineering and prediction acceptance.
c=makeAbfdConfig(c);
required={'engineeringPassed','budgetPassed','onlinePassed','tubeCoverage95'};
assert(all(isfield(metrics,required)),'ABFD:MissingAcceptanceMetrics', ...
    'Engineering, force budget, P95 and tube coverage metrics must be recorded.');
for name=required(1:3)
    value=metrics.(name{1});
    assert(isscalar(value) && (islogical(value) || isnumeric(value)) && ...
        isfinite(value) && any(value==[0,1]),'ABFD:InvalidAcceptanceMetric','Invalid %s.',name{1});
end
coverage=metrics.tubeCoverage95;
% MATLAB decodes an explicit JSON null in a scalar field as numeric [].
if isnumeric(coverage) && isempty(coverage), coverage=NaN; end
assert(isnumeric(coverage) && isscalar(coverage) && (isnan(coverage) || ...
    (isfinite(coverage) && coverage>=0 && coverage<=1)), ...
    'ABFD:InvalidCoverage','Coverage must be a fraction or explicitly unavailable (NaN).');
tubeRequired=any(strcmp(c.controllerMode,{'geometric-D','fullstate-tvlqr'})) && ...
    ~any([c.qpEnabled,c.dobEnabled,c.governorEnabled,c.antiWindupEnabled]);
tubeApplicable=tubeRequired || isfinite(coverage);
tubePassed=~tubeApplicable || (isfinite(coverage) && coverage>=.95);
tubeStatus='not-applicable';
if tubeApplicable
    tubeStatus='failed'; if tubePassed, tubeStatus='passed'; end
    if isnan(coverage), tubeStatus='missing'; end
end
reasons={};
for name=required(1:3)
    if ~metrics.(name{1}), reasons{end+1}=name{1}; end %#ok<AGROW>
end
if ~tubePassed, reasons{end+1}=['tube-',tubeStatus]; end
acceptance=struct('schemaVersion',1,'engineeringPassed',logical(metrics.engineeringPassed), ...
    'budgetPassed',logical(metrics.budgetPassed),'onlinePassed',logical(metrics.onlinePassed), ...
    'tubeRequired',tubeRequired,'tubeApplicable',tubeApplicable,'tubeStatus',tubeStatus, ...
    'tubeCoverage95',coverage,'requiredTubeCoverage',.95,'passed',isempty(reasons),'reasons',{reasons}, ...
    'scope','Engineering constraints, common force budget, online P95, and applicable joint state-tube coverage; no cross-case superiority claim');
end
