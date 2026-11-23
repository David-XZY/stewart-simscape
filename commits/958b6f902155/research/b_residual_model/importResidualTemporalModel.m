function model=importResidualTemporalModel(file,staticFile,allowPilot,provider)
if nargin<3, allowPilot=false; end
if nargin<4, provider=[]; end
model=jsondecode(fileread(file));
assert(model.schemaVersion==1 && strcmp(model.kind,'standardized-residual-var'), ...
    'ABFD:TemporalSchema','Invalid temporal model schema.');
assert(allowPilot && strcmp(model.status,'calibrated-pilot'), ...
    'ABFD:TemporalPilot','This temporal artifact has not passed full coverage acceptance; explicit pilot use required.');
assert(strcmp(model.staticAssetHash,abfdFileHash(staticFile)), ...
    'ABFD:TemporalAssetMismatch','Temporal and static residual assets differ.');
validateattributes(model.order,{'double'},{'scalar','integer','positive'}); n=6*model.order;
validateattributes(model.sampleTime,{'double'},{'scalar','positive','finite'});
validateattributes(model.standardizedMean,{'double'},{'numel',6,'real','finite'});
model.standardizedMean=model.standardizedMean(:);
for name={'transition','processCovariance','initialCovariance'}
    a=model.(name{1}); validateattributes(a,{'double'},{'size',[n,n],'real','finite'});
    if ~strcmp(name{1},'transition')
        assert(norm(a-a.','fro')<1e-8 && min(eig(a))>=-1e-9, ...
            'ABFD:TemporalCovariance','Invalid temporal covariance.');
    end
end
A=model.transition; Q=model.processCovariance; P=model.initialCovariance;
assert(max(abs(eig(A)))<1,'ABFD:TemporalUnstable','Temporal model must be stable.');
assert(norm(P-A*P*A.'-Q,'fro')<1e-8*max(1,norm(P,'fro')), ...
    'ABFD:TemporalStationary','Initial covariance does not match temporal dynamics.');
assert(norm(Q(7:end,:),'fro')<1e-10 && ...
    (model.order==1 || norm(A(7:end,:)-[eye(n-6),zeros(n-6,6)],'fro')<1e-10), ...
    'ABFD:TemporalCompanion','Invalid history shift or innovation location.');
chol(P); model.assetHash=abfdFileHash(file);
if isfield(model,'conditioning')
    model.staticNetworkHash=abfdHash(importResidualNetworkWeights(staticFile));
end
validateAbfdTemporalConditioning(provider,model);
end
