function test_diffusion_core()
c=setupDiffusion(); failures={};
files=dir(fullfile(c.root,'diffusion','*.m'));
for k=1:numel(files)
    messages=checkcode(fullfile(files(k).folder,files(k).name),'-id');
    for j=1:numel(messages)
        if any(strcmp(messages(j).id,{'PARSE','SYNER','ENDCT'})), failures{end+1}=messages(j).message; end %#ok<AGROW>
    end
end
assert(isempty(failures),'DP:Syntax','MATLAB parsing failed.');
task=makeTask('hood',20260915,'development',true); base=dpBaseline(task);
assert(task.scene.collisionGeometry.safeDistance==.01 && task.scene.collisionGeometry.finalGap==.01);
assert(abs(task.scene.target.roofGoalGap-.01)<1e-12);
tr=dpSpline(base,[0,task.T],task.T);
assert(norm(tr.q(:,1)-task.q0)<1e-12 && norm(tr.q(:,2)-task.qGoal)<1e-12);
assert(max(abs([tr.v(:);tr.a(:)]))<1e-10);
t=3; e=1e-4; b=dpSpline(base,[t-e,t,t+e],task.T);
assert(norm((b.q(:,3)-b.q(:,1))/(2*e)-b.v(:,2))<1e-7);
assert(norm((b.v(:,3)-b.v(:,1))/(2*e)-b.a(:,2))<1e-7);
slow=dpSpline(base,2*t,2*task.T); assert(norm(slow.v-b.v(:,2)/2)<1e-12 && norm(slow.a-b.a(:,2)/4)<1e-12);
m=task.model; p=dpPhysics(m); q=(task.q0+task.qGoal)/2; v=[.02;-.01;.04;.02;.03;-.02]; a=[.03;-.04;.05;-.02;.01;.04];
[L,Ld,Ldd,F,J,W]=p.point(q,v,a); kin=computeLegKinematics(q,v,a,m); wn=computeCompositeRequiredWrench(q,v,a,m);
assert(max(abs(full(L)-kin.L))<1e-10 && max(abs(full(Ld)-kin.Ld))<1e-10);
assert(max(abs(full(Ldd)-kin.Ldd))<1e-10 && max(abs(full(W)-wn))<1e-8);
assert(norm(full(J).'*full(F)-wn)<1e-7);
[~,~,~,high]=p.point(q,zeros(6,1),[0;0;100;0;0;0]); assert(max(abs(full(high)))>2000,'DP:Clip','Demand force was clipped.');
d=dpClearance(task.qGoal,task); assert(abs(min(d)-.01)<1e-9);
near=task.qGoal; near(1:3)=near(1:3)+task.scene.box.R_S(:,3)*1e-4;
assert(min(dpClearance(near,task))<.01-1e-5);
penetrating=task.qGoal; penetrating(1:3)=penetrating(1:3)+task.scene.box.R_S(:,3)*.02;
assert(min(dpClearance(penetrating,task))<0);
% End-cap and side-wall distances have analytic answers for aligned boxes.
geometry=task; R=rpy2rotmZYX(task.q0(4:6));
center=task.q0(1:3)+R*task.scene.objectCylinder.center_P;
for direction=1:2
    support=task.scene.objectCylinder.length/2;
    if direction==2, support=task.scene.objectCylinder.radius; end
    for gap=[.01,.01-1e-6,-.001]
        obstacle=task.scene.hood.obstacles(1); obstacle.R_S=R; obstacle.halfSize=.05*ones(3,1);
        obstacle.center_S=center+R(:,direction)*(support+.05+gap);
        geometry.scene.hood.obstacles=obstacle;
        assert(abs(dpClearance(task.q0,geometry,true)-gap)<1e-8,'DP:CapSide','Cylinder cap/side gap mismatch.');
    end
end
unreachable=makeTask(struct('family','hood','positionOffset',[5;0;0]),20260915,'development',true);
assert(~unreachable.endpointPassed && ~isempty(unreachable.endpointReason));
offset=[.04;-.04;.04]; angle=deg2rad([8;-8;8]);
corner=makeTask(struct('family','hood','positionOffset',offset,'attitudeOffset',angle),20260915,'development',true);
assert(norm(corner.qGoal-corner.nominalGoal-[offset;angle])<1e-12,'DP:GoalRange','Goal range must describe the platform endpoint.');
simple=makeTask('simple',20260915,'development',true); assert(simple.endpointPassed);
asset=struct('outputMean',zeros(60,1),'outputScale',.05*ones(60,1)); fun=dpGuidance(simple,asset,c);
z=linspace(-.02,.03,60).'; [value,grad]=fun(z); grad=full(grad); fd=zeros(60,1);
for k=1:60
    plus=z; minus=z; plus(k)=plus(k)+1e-5; minus(k)=minus(k)-1e-5;
    fd(k)=(full(fun(plus))-full(fun(minus)))/(2e-5);
end
assert(norm(fd-grad)/max(1,norm(fd))<2e-3,'DP:Gradient','Guidance gradient mismatch.');
assert(isfinite(full(value)));
% All-invalid candidates must remain failed, without invoking the optimizer.
bad=c; bad.output=fullfile(c.output,'invalid_candidate_test');
rejected=refineAndValidate(simple,NaN(6,16,8),bad);
assert(~rejected.passed && numel(rejected.attempts)==8 && all(~[rejected.attempts.passed]));
% Task duration is a condition, not silently repaired to an easier time.
short=makeTask(struct('family','simple','duration',.2),20260915,'development',true);
assert(short.T==.2);
fast=dpSpline(dpBaseline(short),linspace(0,.2,61),.2); [~,~,~,forceFast]=p.point(fast.q,fast.v,fast.a);
assert(max(abs(full(forceFast(:))))>2000);
shortReport=dpValidate(struct('kind','spline','cp',dpBaseline(short),'T',short.T),short,c);
assert(~shortReport.passed);
% A load trapped inside a solid box is rejected by posterior geometry checks.
blocked=simple; obstacle=blocked.scene.hood.obstacles(1);
obstacle.center_S=center; obstacle.R_S=eye(3); obstacle.halfSize=2*ones(3,1);
blocked.scene.hood.obstacles=obstacle;
blockedReport=dpValidate(struct('kind','spline','cp',dpBaseline(blocked),'T',blocked.T),blocked,c);
assert(~blockedReport.passed && blockedReport.minClearance<0);
stationary=simple; stationary.qGoal=stationary.q0; stationary.T=.02;
[~,~,~,staticForce]=p.point(stationary.q0,zeros(6,1),zeros(6,1));
bundle=struct('kind','ihsid','T',.02,'intervals',1,'h',.02,'tNode',[0,.02], ...
    'X',repmat([stationary.q0;zeros(6,1)],1,3),'A',zeros(6,3),'F',repmat(full(staticForce),1,3));
validStatic=dpValidate(bundle,stationary,c); assert(validStatic.passed);
bundle.F(1,1)=NaN; invalidForce=dpValidate(bundle,stationary,c); assert(~invalidForce.passed);
report=struct('passed',true,'spline',true,'physicsNumericCrosscheck',true,'forceUnclipped',true, ...
    'clearanceBoundary',true,'capAndSideGeometry',true,'unreachableEndpoint',true,'blockedGeometry',true, ...
    'shortDurationRejected',true,'allInvalidCandidatesRejected',true,'nonfiniteInterpolatedForceRejected',true, ...
    'gradientRelativeError',norm(fd-grad)/max(1,norm(fd)));
dpWriteJson(fullfile(c.output,'core_tests.json'),report);
fprintf('DP_CORE_TESTS_PASS\n');
end
