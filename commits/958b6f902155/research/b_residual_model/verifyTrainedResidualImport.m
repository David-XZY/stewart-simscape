function report=verifyTrainedResidualImport(modelFile,referenceFile,model,initialPose,provider)
import casadi.*
if nargin<4, initialPose=[]; end
net=importResidualNetworkWeights(modelFile);
reference=jsondecode(fileread(referenceFile));
assert(strcmp(reference.assetHash,abfdFileHash(modelFile)), ...
    'ABFD:VerificationAsset','Torch reference belongs to different weights.');
if nargin<5
    provider=buildDynamicsProvider(model,'residual-B',net,false,initialPose);
else
    assert(strcmp(abfdHash(provider.network),abfdHash(net)) && strcmp(abfdHash(provider.nominalModel),abfdHash(model)), ...
        'ABFD:VerificationProvider','Supplied task provider uses a different model or network.');
end
ad=buildCasadiCompositeDynamics(provider);
z=SX.sym('import_state',12); [mu,Sigma]=residualMlpCasadiExpr(z,net);
evaluate=Function('import_verification',{z},{mu,Sigma,jacobian(mu,z)});
report=struct('passed',false,'samples',numel(reference.samples),'assetHash',reference.assetHash, ...
    'maxMeanError',0,'maxVarianceRelativeError',0,'maxJacobianRelativeError',0, ...
    'maxDynamicsError',0,'scope',net.trainingScope);
for k=1:numel(reference.samples)
    sample=reference.samples(k); x=sample.x(:);
    [numericMean,numericVariance]=evaluateResidualMlp(x,net);
    [symbolicMean,symbolicVariance,J]=evaluate(x);
    report.maxMeanError=max([report.maxMeanError;abs(numericMean-sample.mean(:));abs(full(symbolicMean)-sample.mean(:))]);
    report.maxVarianceRelativeError=max(report.maxVarianceRelativeError, ...
        max(abs([diag(numericVariance),diag(full(symbolicVariance))]-sample.variance(:)),[],'all')/max(1,max(sample.variance)));
    report.maxJacobianRelativeError=max(report.maxJacobianRelativeError, ...
        norm(full(J)-sample.meanJacobian,'fro')/max(1,norm(sample.meanJacobian,'fro')));
    a=.1*sin((1:6).'+k); F=provider.inverse(x(1:6),x(7:12),a);
    numeric=provider.forward(x,F); symbolic=full(ad.f(x,F));
    report.maxDynamicsError=max([report.maxDynamicsError;abs(numeric-[x(7:12);a]);abs(numeric-symbolic)]);
end
report.passed=report.maxMeanError<1e-8 && report.maxVarianceRelativeError<1e-10 && ...
    report.maxJacobianRelativeError<1e-10 && report.maxDynamicsError<1e-8;
assert(report.passed,'ABFD:TrainedImportMismatch','Trained Python/MATLAB/CasADi import comparison failed.');
end
