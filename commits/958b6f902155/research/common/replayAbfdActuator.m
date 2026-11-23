function prediction=replayAbfdActuator(commandTime,command,queryTime,initialForce,profile,model)
% Replay the existing Simscape block order: delay -> dead zone -> slew limit
% -> lag -> physical output saturation. Commands themselves remain unchanged.
t=commandTime(:).'; query=queryTime(:).'; initialForce=initialForce(:);
assert(all(diff(t)>0) && all(diff(query)>0) && t(1)==0 && query(1)==0, ...
    'ABFD:ActuatorReplayTime','Use increasing recorded times starting at zero.');
assert(isequal(size(command),[6,numel(t)]) && all(isfinite(command(:))), ...
    'ABFD:ActuatorReplayShape','Invalid command force log.');
tau=profile.forceLag; delay=profile.forceDelay; rate=profile.forceRate*ones(6,1);
validateattributes(tau,{'double'},{'scalar','positive','finite'});
validateattributes(rate,{'double'},{'positive','finite'});
events=unique([0,query,t+delay]); events=events(events<=query(end));
state=struct('slew',initialForce,'lag',initialForce); forces=zeros(6,numel(query));
limit=max(abs([model.actuator.forceMin(:),model.actuator.forceMax(:)]),[],2);
maximumRamp=0; saturationCount=0;
for k=1:numel(events)
    time=events(k); index=find(t<=time-delay+1e-12,1,'last');
    input=initialForce; if ~isempty(index), input=command(:,index); end
    target=sign(input).*max(abs(input)-profile.forceDeadzone,0);
    ids=find(abs(query-time)<1e-12);
    output=min(max(state.lag,-limit),limit);
    if ~isempty(ids), forces(:,ids)=repmat(output,1,numel(ids)); end
    saturationCount=saturationCount+sum(abs(state.lag)>limit);
    if k<numel(events)
        h=events(k+1)-time;
        [state,~,rampTime]=advanceAbfdActuator(state,target,h,profile,model);
        maximumRamp=max(maximumRamp,max(rampTime));
    end
end
prediction=struct('t',query,'actualForce',forces,'maximumRampDuration',maximumRamp, ...
    'saturationEventCount',saturationCount,'profileHash',abfdHash(profile), ...
    'model','Exact piecewise-linear slew and first-order lag; existing Simscape actuator block order');
end
