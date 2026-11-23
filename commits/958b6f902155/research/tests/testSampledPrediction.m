function report=testSampledPrediction()
% Compare to independent matrix-exponential sample/hold covariance, including
% off-clock knots and a nonzero held wrench fixture (never a trained model).
import casadi.*
m=buildOptModelCustom(); n=struct('schemaVersion',1,'kind','residual-wrench-tanh','status','test-fixture', ...
    'physical',struct('mass',m.dynamics.totalMass,'com',m.dynamics.comP,'inertia',m.dynamics.inertiaAtCOM_P), ...
    'inputMean',zeros(12,1),'inputScale',ones(12,1),'outputScale',ones(6,1),'varianceCalibration',1, ...
    'layers',struct('W',zeros(12),'b',zeros(12,1)));
p=buildDynamicsProvider(m,'residual-B',n,true); ad=buildCasadiCompositeDynamics(p);
x=[m.qHome;zeros(6,1)]; F=p.inverse(x(1:6),x(7:12),zeros(6,1)); o=p.evaluate(x(1:6),x(7:12),zeros(6,1));
K=o.Gamma*[25*eye(6),10*eye(6)]; C=-K;
c=makeAbfdConfig(struct('predictionProfile','sampled-hold-v2','processSpectralDensity',zeros(12)));
design=struct('kind','force','schedule',struct('t',0:.01:.1,'K',repmat(K,1,1,10)));
times=[0,.003,.01,.017,.02,.1];
[~,tight,d]=buildAbfdTrackingExpressions(repmat(x,1,numel(times)),zeros(6,numel(times)), ...
    repmat(F,1,numel(times)),times,p,design,c,struct('hood',struct('obstacles',[])));
fun=Function('sampled_prediction_test',{},[d.P,d.heldForceCovariance,d.legAccelerationVariance,{tight}]); values=fun.call({});
[fx,fu]=ad.linearize(x,F); fx=full(fx); fu=full(fu); G=-[zeros(6);o.H\eye(6)];
[Ad,Bd]=discretizeAbfdZoh(fx,[fu,G],.01);
exact=expm([fx,fu,G;zeros(12,24)]*.01);
zohError=norm([Ad,Bd]-exact(1:12,:),'fro'); assert(zohError<1e-10);
P=blkdiag(c.initialCovariance,zeros(12)); reset=[eye(12),zeros(12);C,zeros(6,12);zeros(6,24)];
reference=cell(size(times)); cursor=0; maximumError=0;
for j=1:numel(times)
    if j==1, P=reset*P*reset.'+blkdiag(zeros(18),o.Sigma); end
    while cursor<times(j)-1e-12
        tick=(floor(cursor/.01+1e-9)+1)*.01; finish=min(tick,times(j));
        Phi=expm([fx,fu,G;zeros(12,24)]*(finish-cursor));
        % Last 12 coordinates have zero derivatives, hence identity hold.
        P=Phi*P*Phi.'; cursor=finish;
        if abs(cursor/.01-round(cursor/.01))<1e-8
            P=reset*P*reset.'+blkdiag(zeros(18),o.Sigma);
        end
    end
    reference{j}=P(1:12,1:12);
    maximumError=max(maximumError,norm(full(values{j})-reference{j},'fro')/norm(reference{j},'fro'));
end
assert(maximumError<1e-9);
[~,~,coarse]=buildAbfdTrackingExpressions(repmat(x,1,2),zeros(6,2),repmat(F,1,2),[0,.1],p,design,c,struct('hood',struct('obstacles',[])));
coarseFun=Function('coarse_covariance',{},coarse.P); coarseValues=coarseFun.call({});
partitionError=norm(full(coarseValues{2})-reference{end},'fro')/norm(reference{end},'fro');
assert(partitionError<1e-9);
leg=buildAbfdLegAccelerationAD(ad); [~,~,acc,ax,af,aw]=leg(x,F);
ax=full(ax); af=full(af); aw=full(aw); finite=zeros(6,18); h=1e-5;
for j=1:18
    delta=zeros(18,1); delta(j)=h;
    finite(:,j)=(physicalLegAcceleration(p,x+delta(1:12),F+delta(13:18))- ...
        physicalLegAcceleration(p,x-delta(1:12),F-delta(13:18)))/(2*h);
end
jacobianError=norm([ax,af]-finite,'fro')/norm(finite,'fro'); assert(jacobianError<1e-6);
variance=diag((ax+af*C)*c.initialCovariance*(ax+af*C).'+aw*o.Sigma*aw.');
assert(norm(variance-full(values{2*numel(times)+1}))<1e-8);
tight=full(values{end}); expected=[full(acc);-full(acc)]+c.tubeSigma*repmat(sqrt(variance+1e-16),2,1)-repmat(m.actuator.lddotMax,2,1);
assert(norm(tight(37:48)-expected)<1e-8 && numel(tight)==48*numel(times));
% The reference can contain off-clock certification knots without adding gains.
refs=struct('q',repmat(x(1:6),1,7),'qd',zeros(6,7),'qdd',zeros(6,7), ...
    'Fleg',repmat(F,1,7),'t',[0,.003,.01,.017,.02,.027,.03]);
c.controllerMode='fullstate-tvlqr'; sampled=buildAbfdSchedule(refs,p,c);
assert(numel(sampled.schedule.t)==4 && size(sampled.schedule.K,3)==3);
runtime=struct('bundle',struct('refs',refs),'provider',p,'config',c,'design',sampled);
[~,~,info]=stepAbfdController(.01,x,struct(),runtime);
assert(norm(info.gain-sampled.schedule.K(:,:,2),'fro')<1e-12);
old=rmfield(c,'predictionProfile'); legacy=buildAbfdSchedule(refs,p,old);
assert(size(legacy.schedule.K,3)==6);
report=struct('passed',true,'zohAbsoluteError',zohError,'covarianceRelativeError',maximumError, ...
    'offClockPartitionRelativeError',partitionError,'accelerationJacobianRelativeError',jacobianError, ...
    'legacyGainCount',size(legacy.schedule.K,3),'sampledGainCount',size(sampled.schedule.K,3));
end
function value=physicalLegAcceleration(p,x,F)
xd=p.forward(x,F); kin=computeLegKinematics(x(1:6),x(7:12),xd(7:12),p.model); value=kin.Ldd;
end
