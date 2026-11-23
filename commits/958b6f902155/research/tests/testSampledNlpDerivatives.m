function report=testSampledNlpDerivatives()
% Short stationary fixture tests the composed A+D NLP derivatives, not task
% feasibility. Physical task constraints remain present in the constructed NLP.
import casadi.*
m=buildOptModelCustom(); s=buildCylinderBoxTransferScene(m);
s.q0=m.qHome; s.qWaypoint=m.qHome; s.qGoal=m.qHome;
s.qd0=zeros(6,1); s.qdGoal=zeros(6,1);
s.phase.durationApproach=.03; s.phase.durationInsertion=.015;
d=buildTwoPhaseIHSDiscretization(s,2,1);
[z,initial]=buildInitialGuessTwoPhaseIHSImplicit(m,s,d); p=buildDynamicsProvider(m);
c=makeAbfdConfig(struct('predictionProfile','sampled-hold-v2','controllerMode','geometric-D', ...
    'dVariant','fixed','designMode','codesign-A'));
raw=struct('t',d.tNode,'tc',d.tMid,'Xnode',initial.Xnode,'Xmid',initial.XmidIHS,'Anode',initial.Anode,'Amid',initial.Amid);
refs=reconstructIhsidContinuousReference(raw,p,c.sampleTime);
design=buildAbfdSchedule(refs,p,c); design.nominal=refs;
research=struct('provider',p,'config',c,'design',design,'center',[],'buildOnly',true);
timer=tic; nlp=buildCasadiTrackableIHSNLP(m,s,d,initial,struct(),research);
symbol=MX.sym('check_decision',numel(z)); [J,~,inequality]=nlp.eval(symbol);
stream=RandStream('mt19937ar','Seed',71); direction=randn(stream,numel(z),1); direction=direction/norm(direction);
derivative=Function('sampled_nlp_directional',{symbol},{jtimes([J;inequality],symbol,direction)});
analytic=full(derivative(z)); steps=[1e-4,1e-5,1e-6,1e-7]; errors=zeros(size(steps));
for k=1:numel(steps)
    h=steps(k); [jp,~,cp]=nlp.eval(z+h*direction); [jm,~,cm]=nlp.eval(z-h*direction);
    finite=full([jp-jm;cp-cm])/(2*h); errors(k)=norm(analytic-finite)/max(1,norm(finite));
    [~,worst]=max(abs(analytic-finite));
    fprintf('h=%g, relative=%g, nonfinite AD=%d, worst row=%d: AD=%g FD=%g\n', ...
        h,errors(k),sum(~isfinite(analytic)),worst,analytic(worst),finite(worst));
end
error=errors(end);
assert(all(isfinite(analytic)) && error<1e-5,'ABFD:SampledNlpDerivative','Sampled A+D NLP derivative mismatch.');
report=struct('passed',true,'relativeError',error,'steps',steps,'errors',errors,'seconds',toc(timer),'decisionCount',numel(z), ...
    'scope','stationary short-horizon fixture; full A+D sampled expressions and acceleration tightening differentiated; no optimization acceptance claim');
end
