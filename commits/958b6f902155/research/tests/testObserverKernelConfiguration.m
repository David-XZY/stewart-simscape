function report=testObserverKernelConfiguration()
% Keep old configuration fields/hashes, and reject ambiguous kernel selection.
original=makeAbfdConfig();
assert(~isfield(original,'observerKernel') && isequaln(original,makeAbfdConfig(original)), ...
    'ABFD:ObserverKernelDefault','Default configuration must retain its original shape.');
for name={'interval-rk4-v1','fused-rk4-v1'}
    option=struct('stateSource','external-observer','observerKernel',name{1});
    c=makeAbfdConfig(option);
    assert(strcmp(c.observerKernel,name{1}),'ABFD:ObserverKernelSelection','Kernel name changed.');
end
invalid={struct('observerKernel','fused-rk4-v1'), ...
    struct('stateSource','external-observer','observerKernel','fused'), ...
    struct('stateSource','external-observer','observerKernel',''), ...
    struct('stateSource','external-observer','observerKernel',{{'interval-rk4-v1','unsupported'}})};
for k=1:numel(invalid)
    rejected=false;
    try
        makeAbfdConfig(invalid{k});
    catch e
        rejected=strcmp(e.identifier,'ABFD:ObserverKernelConfig');
    end
    assert(rejected,'ABFD:ObserverKernelRefusal','Invalid kernel configuration %d was not rejected.',k);
end
report=struct('passed',true,'acceptedExplicitKernels',2,'rejectedConfigurations',numel(invalid), ...
    'scope','Optional configuration and legacy default shape; no numerical or Simscape acceptance');
end
