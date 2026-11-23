function [q,qd] = ch5AbsoluteState(relative,velocity,framePose)
% The sensor base is frame A. Its rotation, rather than Euler coordinates,
% composes with the world-to-A transform. Velocity is resolved in base A.
R0=ch5Rotation(framePose(4:6));R=R0*ch5Rotation(relative(4:6));
q=[framePose(1:3)+R0*relative(1:3);ch5Rotation(R)];
E=rpyRateMapZYX(q(4:6));
assert(rcond(E)>1e-9,'ch5:EulerChart','Pose is outside the supported ZYX chart.');
qd=[R0*velocity(1:3);E\(R0*velocity(4:6))];
end
