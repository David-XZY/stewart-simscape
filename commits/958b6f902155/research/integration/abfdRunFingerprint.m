function value=abfdRunFingerprint(task,c)
residual=''; proposal='';
if strcmp(c.modelMode,'residual-B'), residual=abfdFileHash(c.residualFile); end
if strcmp(c.plannerMode,'learned-warmstart-F'), proposal=abfdFileHash(c.proposalFile); end
parts=struct('task',task,'config',c,'residual',residual, ...
    'proposal',proposal,'trajectory',abfdFileHash(c.trajectoryFile));
if isfield(c,'staticBalanceFile'), parts.staticBalance=abfdFileHash(c.staticBalanceFile); end
if isfield(c,'inertiaCalibrationFile'), parts.startupInertia=abfdFileHash(c.inertiaCalibrationFile); end
if isfield(c,'temporalFile'), parts.temporal=abfdFileHash(c.temporalFile); end
if isfield(c,'plantInitializationFile'), parts.plantInitialization=abfdFileHash(c.plantInitializationFile); end
value=abfdHash(parts);
end
