function report=testConditionalTemporalRejection(staticFile,legacyFile,conditionalFile)
% Conditional pilot artifacts must not be reused by an unconditioned rollout.
legacy=importResidualTemporalModel(legacyFile,staticFile,true);
rejected=false;
try
    importResidualTemporalModel(conditionalFile,staticFile,true);
catch e
    rejected=strcmp(e.identifier,'ABFD:TemporalConditioningMismatch');
end
assert(rejected,'ABFD:ConditionalTemporalGuard','Conditional artifact was silently imported.');
conditional=jsondecode(fileread(conditionalFile)); rejectedKernel=false;
try
    buildAbfdActuatorRolloutAD([],[],[],[],[],[],[],conditional,[]);
catch e
    rejectedKernel=strcmp(e.identifier,'ABFD:TemporalConditioningMismatch');
end
assert(rejectedKernel,'ABFD:ConditionalTemporalGuard','Conditional artifact bypassed import through the low-level kernel.');
report=struct('passed',true,'legacyOrder',legacy.order,'conditionalImportRejected',rejected, ...
    'conditionalKernelRejected',rejectedKernel,'scope','Unbound conditional propagation is rejected; this check is not coverage acceptance');
end
