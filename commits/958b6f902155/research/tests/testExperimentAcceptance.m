function testExperimentAcceptance()
c=makeAbfdConfig(struct('controllerMode','geometric-D'));
m=struct('engineeringPassed',true,'budgetPassed',true,'onlinePassed',true,'tubeCoverage95',.98);
a=assessAbfdExperimentMetrics(m,c); assert(a.passed && a.tubeRequired);
m.onlinePassed=false; a=assessAbfdExperimentMetrics(m,c); assert(~a.passed);
m.onlinePassed=true; m.tubeCoverage95=.94;
a=assessAbfdExperimentMetrics(m,c); assert(~a.passed && strcmp(a.tubeStatus,'failed'));
m.tubeCoverage95=NaN; a=assessAbfdExperimentMetrics(m,c);
assert(~a.passed && strcmp(a.tubeStatus,'missing'));
c.controllerMode='baseline-lqi'; a=assessAbfdExperimentMetrics(m,c);
assert(a.passed && ~a.tubeApplicable && strcmp(a.tubeStatus,'not-applicable'));
serialized=jsondecode(jsonencode(m)); a=assessAbfdExperimentMetrics(serialized,c);
assert(a.passed && ~a.tubeApplicable && isnan(a.tubeCoverage95));
c.controllerMode='geometric-D'; a=assessAbfdExperimentMetrics(serialized,c);
assert(~a.passed && strcmp(a.tubeStatus,'missing')); c.controllerMode='baseline-lqi';
m.onlinePassed=false; a=assessAbfdExperimentMetrics(m,c); assert(~a.passed);
m.onlinePassed=true; m.tubeCoverage95=1.01;
expectError(@()assessAbfdExperimentMetrics(m,c),'ABFD:InvalidCoverage');

base=makeAbfdConfig(struct('plannerProfile','guarded-adaptive-v2','predictionProfile','sampled-hold-v2'));
matrix=buildAbfdExperimentMatrix(base); assert(numel(matrix)==13);
b=struct('predictionProfile','actuator-var-v3','temporalFile','temporal.json', ...
    'staticBalanceFile','observation.json','inertiaCalibrationFile','observation.json');
overrides=struct('EB',b,'EABD',b,'EABFD',b);
overrides.EABD.solverFormulation='covariance-shooting';
overrides.EABD.solverDerivativeAssembly='grouped-moments';
matrix=buildAbfdExperimentMatrix(base,overrides);
for k=1:numel(matrix)
    config=matrix(k).config;
    if strcmp(config.modelMode,'analytic')
        assert(~any(isfield(config,{'staticBalanceFile','inertiaCalibrationFile','temporalFile'})));
        assert(strcmp(config.predictionProfile,'sampled-hold-v2'));
    else
        assert(strcmp(config.predictionProfile,'actuator-var-v3'));
        assert(strcmp(config.staticBalanceFile,'observation.json'));
    end
end
assert(strcmp(matrix(strcmp({matrix.id},'EABD')).config.solverDerivativeAssembly,'grouped-moments'));
bad=base; bad.staticBalanceFile='observation.json';
expectError(@()buildAbfdExperimentMatrix(bad),'ABFD:MatrixScopedOption');
expectError(@()buildAbfdExperimentMatrix(base,struct('EB',struct('feedbackBudget',121))), ...
    'ABFD:MatrixComparisonChanged');
expectError(@()buildAbfdExperimentMatrix(base,struct('EA',b)),'ABFD:MatrixCaseConfig');
expectError(@()buildAbfdExperimentMatrix(base,struct('TYPO',struct())),'ABFD:MatrixCaseOverrides');
shared=base; shared.plantInitializationFile='shared-observation.json';
matrix=buildAbfdExperimentMatrix(shared,overrides);
assert(all(arrayfun(@(item)strcmp(item.config.plantInitializationFile,shared.plantInitializationFile),matrix)));
expectError(@()buildAbfdExperimentMatrix(base,struct('EB',struct('plantInitializationFile','different.json'))), ...
    'ABFD:MatrixComparisonChanged');
tasks=struct('taskId','fixture');
expectError(@()run_abfd_matrix(tasks,base,struct(),struct('taskId','unknown','caseOverrides',struct())), ...
    'ABFD:MatrixTaskOverrides');
end
function expectError(action,id)
rejected=false;
try
    action();
catch e
    assert(strcmp(e.identifier,id),'ABFD:UnexpectedTestFailure','Expected %s, received %s.',id,e.identifier);
    rejected=true;
end
assert(rejected,'ABFD:MissingRejection','Expected %s.',id);
end
