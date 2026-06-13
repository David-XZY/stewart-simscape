function [payload] = initializePayload(args)
% initializePayload - Initialize the Payload that can then be used for simulations and analysis
%
% Syntax: [payload] = initializePayload(args)
%
% Inputs:
%    - args - Structure with the following fields:
%        - type - 'none', 'rigid', 'flexible', 'cartesian'
%        - h [1x1] - Height of the CoM of the payload w.r.t {M} [m]
%                    This also the position where K and C are defined
%        - K [6x1] - Stiffness of the Payload [N/m, N/rad]
%        - C [6x1] - Damping of the Payload [N/(m/s), N/(rad/s)]
%        - m [1x1] - Mass of the Payload [kg]
%        - I [3x3] - Inertia matrix for the Payload [kg*m2]
%        - center [3x1] - Payload CoM position with respect to {M} [m]
%        - axis [3x1] - Cylinder axis expressed in {M}
%        - radius/length - Cylinder display geometry [m]
%
% Outputs:
%    - payload - Struture with the following properties:
%        - type - 1 (none), 2 (rigid), 3 (flexible)
%        - h [1x1] - Height of the CoM of the payload w.r.t {M} [m]
%        - K [6x1] - Stiffness of the Payload [N/m, N/rad]
%        - C [6x1] - Stiffness of the Payload [N/(m/s), N/(rad/s)]
%        - m [1x1] - Mass of the Payload [kg]
%        - I [3x3] - Inertia matrix for the Payload [kg*m2]

arguments
  args.type char {mustBeMember(args.type,{'none', 'rigid', 'flexible', 'cartesian'})} = 'none'
  args.K (6,1) double {mustBeNumeric, mustBeNonnegative} = 1e8*ones(6,1)
  args.C (6,1) double {mustBeNumeric, mustBeNonnegative} = 1e1*ones(6,1)
  args.h (1,1) double {mustBeNumeric, mustBeNonnegative} = 100e-3
  args.m (1,1) double {mustBeNumeric, mustBeNonnegative} = 10
  args.I (3,3) double {mustBeNumeric, mustBeNonnegative} = 1*eye(3)
  args.center (3,1) double {mustBeNumeric} = nan(3,1)
  args.axis (3,1) double {mustBeNumeric, mustBeFinite} = [0; 0; 1]
  args.radius (1,1) double {mustBeNumeric, mustBePositive} = 0.1
  args.length (1,1) double {mustBeNumeric} = nan
end

switch args.type
  case 'none'
    payload.type = 1;
  case 'rigid'
    payload.type = 2;
  case 'flexible'
    payload.type = 3;
  case 'cartesian'
    payload.type = 4;
end

payload.K = args.K;
payload.C = args.C;
payload.m = args.m;
payload.I = args.I;

payload.h = args.h;
if any(isnan(args.center))
  payload.center = [0; 0; args.h];
else
  validateattributes(args.center, {'double'}, {'finite', 'size', [3, 1]});
  payload.center = args.center;
end
if norm(args.axis) < eps
  error('initializePayload:InvalidAxis', '圆柱载荷轴线必须为非零向量。');
end
payload.axis = args.axis / norm(args.axis);
payload.radius = args.radius;
if isnan(args.length)
  payload.length = 2 * args.h;
else
  validateattributes(args.length, {'double'}, {'scalar', 'positive', 'finite'});
  payload.length = args.length;
end
payload.R = makeCylinderFrame(payload.axis);
payload.I_local = payload.R.' * payload.I * payload.R;
end

function rotation = makeCylinderFrame(axisValue)
% makeCylinderFrame - 构造局部 Z 轴与圆柱轴线重合的右手坐标系
zAxis = axisValue(:) / norm(axisValue);
reference = [0; 1; 0];
if abs(dot(reference, zAxis)) > 0.9
  reference = [0; 0; 1];
end
xAxis = cross(reference, zAxis);
xAxis = xAxis / norm(xAxis);
yAxis = cross(zAxis, xAxis);
rotation = [xAxis, yAxis, zAxis];
end
