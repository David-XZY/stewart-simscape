function geometricTvlqrSimscapeSfun(block)
% Full command replaces the old sum; no additional LQI is placed after D.
block.NumDialogPrms=1; block.NumInputPorts=4; block.NumOutputPorts=7;
for k=1:4
    block.InputPort(k).Dimensions=6; block.InputPort(k).DatatypeID=0;
    block.InputPort(k).Complexity='Real'; block.InputPort(k).SamplingMode='Sample';
    block.InputPort(k).DirectFeedthrough=k<3 || k==4;
end
width=[6,6,6,151,6,6,12];
for k=1:7
    block.OutputPort(k).Dimensions=width(k); block.OutputPort(k).DatatypeID=0;
    block.OutputPort(k).Complexity='Real'; block.OutputPort(k).SamplingMode='Sample';
end
r=block.DialogPrm(1).Data; block.SampleTimes=[r.config.sampleTime,0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@post);
block.RegBlockMethod('InitializeConditions',@init);
block.RegBlockMethod('Outputs',@outputs);
block.RegBlockMethod('Update',@update);
end
function post(b)
width=[6,12,144,6,6,6,6,3,3,6,36,1,1,6,12,144,1];
names={'Integral','ObserverPosterior','ObserverCovariance','PreviousVelocity','AppliedForce','WrenchEstimate', ...
    'PreviousForce','GovernorPosition','GovernorRate','Compensation','PendingControllerState','OutputSeconds','CompletedStepSeconds', ...
    'PreviousForceIntegral','PendingObserverState','PendingObserverCovariance','PreviousObserverTime'};
b.NumDworks=numel(width);
for k=1:numel(width)
    b.Dwork(k).Name=names{k}; b.Dwork(k).Dimensions=width(k); b.Dwork(k).DatatypeID=0;
    b.Dwork(k).Complexity='Real'; b.Dwork(k).UsedAsDiscState=true;
end
end
function init(b)
r=b.DialogPrm(1).Data; b.Dwork(1).Data=zeros(6,1); b.Dwork(2).Data=r.initialState;
b.Dwork(3).Data=reshape(1e-4*eye(12),144,1);
for k=4:17, b.Dwork(k).Data=zeros(b.Dwork(k).Dimensions,1); end
b.Dwork(4).Data=r.initialState(7:12); b.Dwork(5).Data=r.bundle.refs.Fleg(:,1);
if isfield(r,'initialAppliedForce'), b.Dwork(5).Data=r.initialAppliedForce; end
b.Dwork(7).Data=b.Dwork(5).Data;
b.Dwork(15).Data=b.Dwork(2).Data; b.Dwork(16).Data=b.Dwork(3).Data;
end
function [actual,used]=state(b)
r=b.DialogPrm(1).Data;
q=r.poseOrigin+b.InputPort(1).Data; spatial=b.InputPort(2).Data;
E=rpyRateMapZYX(q(4:6)); assert(rcond(E)>1e-5,'ABFD:MeasuredRpySingular','Measured RPY singularity.');
actual=[q;spatial(1:3);E\spatial(4:6)]; used=actual;
if strcmp(r.config.stateSource,'external-observer')
    elapsed=b.CurrentTime-b.Dwork(17).Data;
    assert(elapsed>=0 && elapsed<=r.config.sampleTime+1e-9,'ABFD:ObserverClock','Observer interval is not one controller period.');
    impulse=b.InputPort(4).Data-b.Dwork(14).Data;
    if isfield(r.config,'observerKernel') && strcmp(r.config.observerKernel,'fused-rk4-v1')
        [used,P]=stepAbfdIntervalObserverFused(b.Dwork(2).Data,reshape(b.Dwork(3).Data,12,12),q,impulse,r.provider,elapsed);
    else
        [used,P]=stepAbfdIntervalObserver(b.Dwork(2).Data,reshape(b.Dwork(3).Data,12,12),q,impulse,r.provider,elapsed);
    end
    b.Dwork(15).Data=used; b.Dwork(16).Data=P(:);
end
end
function outputs(b)
r=b.DialogPrm(1).Data; timer=tic; [actual,used]=state(b);
[F,next,info]=stepAbfdController(b.CurrentTime,used,controllerState(b),r);
b.Dwork(11).Data=[next.integral;next.previousVelocity;next.wrenchEstimate;next.previousForce; ...
    next.governorPosition;next.governorRate;next.compensation];
b.OutputPort(1).Data=F; b.OutputPort(2).Data=actual(1:6); b.OutputPort(3).Data=actual(7:12);
b.OutputPort(4).Data=[info.Jv(:);info.Gamma(:);info.gain(:);info.margin(:);b.Dwork(13).Data];
b.OutputPort(5).Data=info.feedforward; b.OutputPort(6).Data=info.feedback;
b.OutputPort(7).Data=used;
b.Dwork(12).Data=toc(timer);
end
function update(b)
timer=tic; r=b.DialogPrm(1).Data; pending=b.Dwork(11).Data;
b.Dwork(1).Data=pending(1:6);
b.Dwork(4).Data=pending(7:12); b.Dwork(5).Data=b.InputPort(3).Data;
b.Dwork(6).Data=pending(13:18); b.Dwork(7).Data=pending(19:24);
b.Dwork(8).Data=pending(25:27); b.Dwork(9).Data=pending(28:30); b.Dwork(10).Data=pending(31:36);
if strcmp(r.config.stateSource,'external-observer')
    b.Dwork(2).Data=b.Dwork(15).Data; b.Dwork(3).Data=b.Dwork(16).Data;
    b.Dwork(14).Data=b.InputPort(4).Data; b.Dwork(17).Data=b.CurrentTime;
end
b.Dwork(13).Data=b.Dwork(12).Data+toc(timer);
end
function s=controllerState(b)
s=struct('integral',b.Dwork(1).Data,'previousVelocity',b.Dwork(4).Data, ...
    'appliedForce',b.Dwork(5).Data,'wrenchEstimate',b.Dwork(6).Data,'previousForce',b.Dwork(7).Data, ...
    'governorPosition',b.Dwork(8).Data,'governorRate',b.Dwork(9).Data,'compensation',b.Dwork(10).Data);
end
