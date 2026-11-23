function create_page2_pose_pid_baseline()
% CREATE_PAGE2_POSE_PID_BASELINE
% Conventional baseline for Page 2: a two-stage Cartesian pose reference
% through the mounting waypoint, then the existing pose-feedback length
% cascade (outer P, inner PIDF) maps it to ideal leg-length actuation.

rootDir = 'D:/NPU/Matlab_code/stewart-simscape';
outDir = 'D:/NPU/Matlab_code/output/page2_trajectory_planning';
addpath(fullfile(rootDir, 'src'));
addpath(fullfile(rootDir, 'opt_minimal', 'core'));
if ~exist(outDir, 'dir'), mkdir(outDir); end
model = buildOptModelCustom();
scene = buildCylinderBoxTransferScene(model);
T1 = 5.0; T2 = 2.5; h = T1 / 40;
t = 0:h:(T1+T2);
n = numel(t); q = zeros(6, n);
R0 = rpy2rotmZYX(scene.q0(4:6));
Rw = rpy2rotmZYX(scene.qWaypoint(4:6));
Rg = rpy2rotmZYX(scene.qGoal(4:6));
for k = 1:n
    if t(k) <= T1 + 1e-12
        s = quintic(t(k) / T1);
        p = (1-s) * scene.q0(1:3) + s * scene.qWaypoint(1:3);
        R = quatToRotm(quatSlerp(rotmToQuat(R0), rotmToQuat(Rw), s));
    else
        s = quintic((t(k)-T1) / T2);
        p = (1-s) * scene.qWaypoint(1:3) + s * scene.qGoal(1:3);
        R = quatToRotm(quatSlerp(rotmToQuat(Rw), rotmToQuat(Rg), s));
    end
    q(:,k) = [p; rotmToRpyZYX(R)];
end
qd = finiteDiff(q, h);
refs = struct('t', t, 'q', q, 'qd', qd, 'q0', scene.q0, ...
    'qWaypoint', scene.qWaypoint, 'qGoal', scene.qGoal, ...
    'description', ['Conventional PID + IK baseline: two-stage Cartesian pose reference ', ...
        'through mounting waypoint; SLERP attitude; phase-wise quintic timing; then existing pose/length PIDF execution.']);
fileName = fullfile(outDir, 'page2_pose_pid_ik_baseline_references.mat');
save(fileName, 'refs');
fprintf('PAGE2_POSE_PID_BASELINE_CREATED %s N=%d\n', fileName, n);
end

function d = finiteDiff(x,h)
n=size(x,2); d=zeros(size(x));
d(:,1)=(-3*x(:,1)+4*x(:,2)-x(:,3))/(2*h); d(:,end)=(3*x(:,end)-4*x(:,end-1)+x(:,end-2))/(2*h);
d(:,2:end-1)=(x(:,3:end)-x(:,1:end-2))/(2*h);
end
function s=quintic(t),t=max(0,min(1,t));s=10*t^3-15*t^4+6*t^5;end
function q=rotmToQuat(R)
tr=trace(R);if tr>0,s=sqrt(tr+1)*2;q=[0.25*s;(R(3,2)-R(2,3))/s;(R(1,3)-R(3,1))/s;(R(2,1)-R(1,2))/s];
elseif R(1,1)>R(2,2)&&R(1,1)>R(3,3),s=sqrt(1+R(1,1)-R(2,2)-R(3,3))*2;q=[(R(3,2)-R(2,3))/s;0.25*s;(R(1,2)+R(2,1))/s;(R(1,3)+R(3,1))/s];
elseif R(2,2)>R(3,3),s=sqrt(1+R(2,2)-R(1,1)-R(3,3))*2;q=[(R(1,3)-R(3,1))/s;(R(1,2)+R(2,1))/s;0.25*s;(R(2,3)+R(3,2))/s];
else,s=sqrt(1+R(3,3)-R(1,1)-R(2,2))*2;q=[(R(2,1)-R(1,2))/s;(R(1,3)+R(3,1))/s;(R(2,3)+R(3,2))/s;0.25*s];end;q=q/norm(q);
end
function q=quatSlerp(q0,q1,s)
q0=q0(:);q1=q1(:);d=dot(q0,q1);if d<0,q1=-q1;d=-d;end;if d>0.9995,q=(1-s)*q0+s*q1;q=q/norm(q);return;end;th=acos(max(-1,min(1,d)));q=(sin((1-s)*th)*q0+sin(s*th)*q1)/sin(th);
end
function R=quatToRotm(q),q=q/norm(q);w=q(1);x=q(2);y=q(3);z=q(4);R=[1-2*(y^2+z^2),2*(x*y-z*w),2*(x*z+y*w);2*(x*y+z*w),1-2*(x^2+z^2),2*(y*z-x*w);2*(x*z-y*w),2*(y*z+x*w),1-2*(x^2+y^2)];end
function rpy=rotmToRpyZYX(R),pitch=asin(max(-1,min(1,-R(3,1))));roll=atan2(R(3,2),R(3,3));yaw=atan2(R(2,1),R(1,1));rpy=[roll;pitch;yaw];end
