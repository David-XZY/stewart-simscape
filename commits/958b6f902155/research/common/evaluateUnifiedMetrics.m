function report=evaluateUnifiedMetrics(log,bundle,c,tube)
if nargin<4, tube=[]; end
t=log.t(:); qref=interp1(bundle.refs.t,bundle.refs.q.',t).';
vref=interp1(bundle.refs.t,bundle.refs.qd.',t).';
error=zeros(12,numel(t)); margins=zeros(6,numel(t)); collision=zeros(1,numel(t));
L=zeros(6,numel(t)); Ld=L; Ldd=L; sigma=zeros(size(t));
for k=1:numel(t)
    error(1:3,k)=log.q(1:3,k)-qref(1:3,k);
    error(4:6,k)=rotationErrorSO3(rpy2rotmZYX(log.q(4:6,k)),rpy2rotmZYX(qref(4:6,k)));
    error(7:12,k)=log.qd(:,k)-vref(:,k);
    p=evaluatePathConstraintsAtPoint([log.q(:,k);log.qd(:,k)],log.actualForce(:,k), ...
        bundle.model,bundle.scene,log.qdd(:,k));
    L(:,k)=p.L; Ld(:,k)=p.Ld; Ldd(:,k)=p.Ldd; sigma(k)=p.sigmaMin;
    required=bundle.scene.collision.safeDistance*ones(size(p.collisionDistances));
    if t(k)>bundle.scene.phase.durationApproach, required(1)=0; end
    collision(k)=max(required-p.collisionDistances);
    margins(:,k)=min(log.actualForce(:,k)-bundle.model.actuator.forceMin, ...
        bundle.model.actuator.forceMax-log.actualForce(:,k));
end
equivalent=[error(1:3,:);bundle.model.Lc*error(4:6,:)];
timing=sort(log.stepSeconds(isfinite(log.stepSeconds) & log.stepSeconds>0));
assert(~isempty(timing),'ABFD:MissingTiming','No completed controller-cycle timings were logged.');
index=max(1,ceil(.95*numel(timing)));
report=struct('translationRms',sqrt(mean(error(1:3,:).^2,2)), ...
    'rotationRms',sqrt(mean(error(4:6,:).^2,2)),'equivalentPoseRms',sqrt(mean(equivalent.^2,'all')), ...
    'translationPeak',max(abs(error(1:3,:)),[],'all'),'rotationPeak',max(abs(error(4:6,:)),[],'all'), ...
    'feedbackForceRms',sqrt(mean(log.feedbackForce.^2,'all')), ...
    'feedbackForcePeak',max(abs(log.feedbackForce),[],'all'), ...
    'actualForcePeak',max(abs(log.actualForce),[],'all'),'forceMargin',min(margins,[],'all'), ...
    'collisionViolationCount',sum(collision>c.constraintTolerance), ...
    'lengthViolationCount',sum(any(L<bundle.model.lmin-c.constraintTolerance | L>bundle.model.lmax+c.constraintTolerance,1)), ...
    'legSpeedPeak',max(abs(Ld),[],'all'),'legAccelerationPeak',max(abs(Ldd),[],'all'), ...
    'minSigma',min(sigma),'onlineP95',timing(index),'tubeCoverage95',NaN,'predictionRmse',NaN);
if ~isempty(tube)
    if isfield(tube,'mean')
        assert(isequal(size(tube.mean),[12,numel(tube.t)]) && all(isfinite(tube.mean(:))), ...
            'ABFD:TubeMean','Invalid predicted mean trajectory.');
        centers=interp1(tube.t,tube.mean.',t).';
        assert(all(isfinite(centers(:))),'ABFD:TubeMeanCoverage','Mean trajectory must cover the complete observation interval.');
        report.tubeCenter='predicted mean';
    end
    covered=false(numel(t),1);
    for k=1:numel(t)
        j=find(tube.t<=t(k),1,'last'); P=tube.P(:,:,j);
        delta=[log.q(:,k)-qref(:,k);log.qd(:,k)-vref(:,k)];
        if isfield(tube,'mean'), delta=[log.q(:,k);log.qd(:,k)]-centers(:,k); end
        % Joint 12D confidence ellipsoid, not marginal 1.96-sigma coverage.
        covered(k)=delta.'*((P+1e-12*eye(12))\delta)<=21.0260698175;
    end
    report.tubeCoverage95=mean(covered);
end
if isfield(log,'predictionError'), report.predictionRmse=sqrt(mean(log.predictionError.^2,'all')); end
report.engineeringPassed=report.forceMargin>=-c.constraintTolerance && report.collisionViolationCount==0 && ...
    report.lengthViolationCount==0 && all(abs(Ld)<=bundle.model.actuator.ldotMax+c.constraintTolerance,'all') && ...
    all(abs(Ldd)<=bundle.model.actuator.lddotMax+c.constraintTolerance,'all') && ...
    report.minSigma>=bundle.model.singularity.sigmaMinSafe-c.constraintTolerance;
report.budgetPassed=report.feedbackForcePeak<=c.feedbackBudget+c.constraintTolerance;
report.onlinePassed=report.onlineP95<=c.onlineP95Limit;
end
