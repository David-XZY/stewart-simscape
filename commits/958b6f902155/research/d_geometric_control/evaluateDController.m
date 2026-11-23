function report=evaluateDController(log,bundle,c)
report=evaluateUnifiedMetrics(log,bundle,c); d=log.diagnostics;
condition=zeros(size(d,1),1); gainNorm=condition; transmissionNorm=condition;
for k=1:size(d,1)
    J=reshape(d(k,1:36),6,6); Gamma=reshape(d(k,37:72),6,6); K=reshape(d(k,73:144),6,12);
    condition(k)=cond(J); gainNorm(k)=norm(K,'fro'); transmissionNorm(k)=norm(Gamma,'fro');
end
report.transmissionConditionRange=[min(condition),max(condition)];
report.gainNormRange=[min(gainNorm),max(gainNorm)];
report.transmissionNormRange=[min(transmissionNorm),max(transmissionNorm)];
report.mechanismAcceptance='Requires paired held-out tracking improvement at the same feedback-force budget.';
end
