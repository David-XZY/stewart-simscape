function log=readAbfdSimscapeLog(simout,setup)
[t,Q]=signal(simout.abfdQ); [tv,V]=signal(simout.abfdQd);
[tf,F]=signal(simout.y.Taum); [tb,FB]=signal(simout.abfdFeedback);
[td,D]=signal(simout.abfdDiagnostics); [tc,C]=signal(simout.u);
time=unique(t); time=time(time>=max([tv(1),tf(1),tb(1),td(1),tc(1)]) & ...
    time<=min([tv(end),tf(end),tb(end),td(end),tc(end)]));
q=interp1(t,Q,time).'; v=interp1(tv,V,time).'; a=zeros(size(v));
for k=1:6, a(k,:)=gradient(v(k,:),time); end
diagnostics=interp1(td,D,time,'previous');
log=struct('t',time.','q',q,'qd',v,'qdd',a, ...
    'actualForce',interp1(tf,F,time).','forceTime',time.', ...
    'commandForce',interp1(tc,C,time).','feedbackForce',interp1(tb,FB,time).', ...
    'stepSeconds',diagnostics(:,151),'diagnostics',diagnostics, ...
    'forceSource',setup.interface.forceSource,'accelerationSource','offline derivative of direct measured generalized velocity');
if isfield(simout,'abfdEstimatedState')
    [te,estimate]=signal(simout.abfdEstimatedState); log.estimatedState=interp1(te,estimate,time).';
end
if isfield(simout,'abfdForceIntegral')
    [ti,impulse]=signal(simout.abfdForceIntegral); log.measuredForceIntegral=interp1(ti,impulse,time).';
end
if isfield(setup.interface,'observerTiming')
    assert(isfield(log,'estimatedState') && isfield(log,'measuredForceIntegral') && ...
        all(isfinite([log.estimatedState(:);log.measuredForceIntegral(:)])), ...
        'ABFD:ObserverLogMissing','Current runtime requires finite estimator and force-integral logs.');
end
assert(all(isfinite([q(:);v(:);a(:);log.actualForce(:)])), ...
    'ABFD:LogInvalid','Nonfinite or missing actual-state/force data.');
end
function [t,x]=signal(s)
assert(isa(s,'timeseries'),'ABFD:LogSignal','Expected a logged timeseries.');
t=s.Time(:); x=squeeze(s.Data); if size(x,1)~=numel(t), x=x.'; end
[t,idx]=unique(t,'last'); x=x(idx,:);
end
