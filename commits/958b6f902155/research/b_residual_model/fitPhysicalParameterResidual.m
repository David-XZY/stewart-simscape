function fit=fitPhysicalParameterResidual(data,model)
% Preserve the original physical bounds, realizable inertia and train-only
% objective. The exact inertial regressor avoids repeating the dynamics loop
% at every finite-difference optimizer evaluation.
fit=fitPhysicalParameterResidualFast(data,model);
folder=fileparts(mfilename('fullpath'));
fit.identification.sourceHash=abfdHash(struct( ...
    'fit',abfdFileHash(fullfile(folder,'fitPhysicalParameterResidualFast.m')), ...
    'regressor',abfdFileHash(fullfile(folder,'buildInertialWrenchRegressor.m'))));
end
