function dynFun = buildCasadiImplicitSolvedDynamics(model)
% buildCasadiImplicitSolvedDynamics - 构造消去加速度后的 CasADi 动力学函数
%
% 给定 X=[q;qd] 和支链力 F，显式求解 A=qdd：
%   H(q,qd)*A = Jv(q)'*F - Wbias(q,qd)
% 返回 xdot=[qd;A]、A 和隐式动力学残差。
import casadi.*
X = MX.sym('X', 12, 1);
F = MX.sym('F', 6, 1);
q = X(1:6);
qd = X(7:12);
kin = ikExpr(q, model);
[Jv, ~] = jacobianExpr(q, model, kin);
Wact = Jv.' * F;
Wbias = wrenchExpr(q, qd, MX.zeros(6, 1), model);
H = MX.zeros(6, 6);
for dofIndex = 1:6
    unitAcceleration = MX.zeros(6, 1);
    unitAcceleration(dofIndex) = 1;
    H(:, dofIndex) = wrenchExpr(q, qd, unitAcceleration, model) - Wbias;
end
A = solve(H, Wact - Wbias);
xdot = [qd; A];
rDyn = Wact - wrenchExpr(q, qd, A, model);
dynFun = Function('implicit_solved_dynamics', {X, F}, {xdot, A, rDyn}, ...
    {'X', 'F'}, {'xdot', 'A', 'rDyn'});
end

function kin = ikExpr(q, model)
import casadi.*
p = q(1:3);
R = rotmZYXExpr(q(4:6));
Bleg = model.B(:, model.legMap);
rB = R * Bleg;
s = MX.zeros(3, 6);
L = MX.zeros(6, 1);
u = MX.zeros(3, 6);
for legIndex = 1:6
    s(:, legIndex) = p + rB(:, legIndex) - model.A(:, legIndex);
    L(legIndex) = sqrt(dot3(s(:, legIndex), s(:, legIndex)));
    u(:, legIndex) = s(:, legIndex) ./ L(legIndex);
end
kin = struct('R', R, 'rB', rB, 's', s, 'L', L, 'u', u);
end

function [Jv, Jq] = jacobianExpr(q, model, kin)
import casadi.*
Jv = MX.zeros(6, 6);
for legIndex = 1:6
    unitDirection = kin.u(:, legIndex);
    upperJointVector = kin.rB(:, legIndex);
    Jv(legIndex, :) = [unitDirection.', cross3(upperJointVector, unitDirection).'];
end
E = rpyRateMapExpr(q(4:6));
T = [eye(3), zeros(3); zeros(3), E];
Jq = Jv * T;
end

function Wreq = wrenchExpr(q, qd, qdd, model)
R = rotmZYXExpr(q(4:6));
[E, Edot] = rpyRateMapExpr(q(4:6), qd(4:6));
omega = E * qd(4:6);
alpha = Edot * qd(4:6) + E * qdd(4:6);
mass = model.dynamics.totalMass;
comWorld = R * model.dynamics.comP;
comAcc = qdd(1:3) + cross3(alpha, comWorld) + cross3(omega, cross3(omega, comWorld));
if model.dynamics.includeGravity
    gravity = model.g;
else
    gravity = zeros(3, 1);
end
force = mass * (comAcc - gravity);
inertiaWorld = R * model.dynamics.inertiaAtCOM_P * R.';
moment = inertiaWorld * alpha + cross3(omega, inertiaWorld * omega) + cross3(comWorld, force);
Wreq = [force; moment];
end

function R = rotmZYXExpr(rpy)
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);
R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
     sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
     -sp,   cp*sr,            cp*cr];
end

function [E, Edot] = rpyRateMapExpr(rpy, rpyDot)
pitch = rpy(2);
yaw = rpy(3);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);
E = [cy*cp, -sy, 0;
     sy*cp,  cy, 0;
     -sp,    0,  1];
if nargout > 1
    thetaDot = rpyDot(2);
    psiDot = rpyDot(3);
    dE_dtheta = [-cy*sp, 0, 0;
                 -sy*sp, 0, 0;
                 -cp,    0, 0];
    dE_dpsi = [-sy*cp, -cy, 0;
                cy*cp, -sy, 0;
                0,      0,  0];
    Edot = dE_dtheta * thetaDot + dE_dpsi * psiDot;
end
end

function c = cross3(a, b)
c = [a(2)*b(3) - a(3)*b(2);
     a(3)*b(1) - a(1)*b(3);
     a(1)*b(2) - a(2)*b(1)];
end

function d = dot3(a, b)
d = a(1)*b(1) + a(2)*b(2) + a(3)*b(3);
end
