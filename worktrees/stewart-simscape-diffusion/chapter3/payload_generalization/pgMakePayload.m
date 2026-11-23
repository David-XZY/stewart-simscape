function p=pgMakePayload(shape,dimensions,mass,mount)
% Uniform rigid payload. SI units; no independent inertia or eccentricity input.
if nargin<4, mount=struct('R',eye(3),'center',[0;0;.15]); end
assert(isequal(size(mount.R),[3,3]) && norm(mount.R-eye(3),'fro')<1e-12 && norm(mount.center(:)-[0;0;.15])<1e-12,'PG:Mount','This protocol freezes the mounting transform.');
d=dimensions(:); assert(isscalar(mass)&&isfinite(mass)&&mass>0 && all(isfinite(d))&&all(d>0),'PG:Payload','Positive finite SI parameters required.');
switch shape
    case 'cylinder'
        assert(numel(d)==2,'PG:Dimensions','Cylinder dimensions are [length,radius].');
        I=diag([mass*d(2)^2/2;mass*(3*d(2)^2+d(1)^2)/12;mass*(3*d(2)^2+d(1)^2)/12]);
        bounds=[d(1);2*d(2);2*d(2)]; volume=pi*d(2)^2*d(1);
    case 'box'
        assert(numel(d)==3,'PG:Dimensions','Box dimensions are [Lx,Ly,Lz].');
        I=mass/12*diag([d(2)^2+d(3)^2;d(1)^2+d(3)^2;d(1)^2+d(2)^2]);
        bounds=d; volume=prod(d);
    otherwise, error('PG:Shape','Unsupported payload shape.');
end
mp=50; Ip=diag([3.125,3.125,6.25]); com=mass/(mp+mass)*mount.center(:);
Ic=Ip+shift(mp,-com)+mount.R*I*mount.R.'+shift(mass,mount.center(:)-com);
assert(min(eig(Ic))>0,'PG:Inertia','Composite inertia must be positive definite.');
p=struct('schemaVersion','pg_payload_v1','shape',shape,'dimensions',d,'mass',mass, ...
    'mount',mount,'boundingDimensions',bounds,'density',mass/volume,'inertiaCOM',I, ...
    'derived',struct('platformMass',mp,'platformInertiaCOM',Ip,'totalMass',mp+mass,'comP',com,'inertiaCOM',Ic));
end
function I=shift(m,r)
I=m*((r.'*r)*eye(3)-r*r.');
end
