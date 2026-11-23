function w=pgDistanceWitness(q,task,j)
p=task.payload; o=task.scene.hood.obstacles(j); R=rpy2rotmZYX(q(4:6))*p.mount.R;
c=q(1:3)+rpy2rotmZYX(q(4:6))*p.mount.center(:); x=c; y=o.center_S;
for k=1:80
    z=o.R_S.'*(x-o.center_S); y=o.center_S+o.R_S*max(-o.halfSize,min(o.halfSize,z));
    z=R.'*(y-c);
    if strcmp(p.shape,'box'), z=max(-p.dimensions/2,min(p.dimensions/2,z));
    else, z(1)=max(-p.dimensions(1)/2,min(p.dimensions(1)/2,z(1))); z(2:3)=z(2:3)*min(1,p.dimensions(2)/max(norm(z(2:3)),realmin)); end
    x=c+R*z;
end
z=R.'*(x-c); b=o.R_S.'*(y-o.center_S);
if strcmp(p.shape,'box'), violation=max(abs(z)-p.dimensions/2);
else, violation=max(abs(z(1))-p.dimensions(1)/2,norm(z(2:3))-p.dimensions(2)); end
violation=max([violation;abs(b)-o.halfSize]);
w=struct('pointPayload',x,'pointObstacle',y,'membershipViolation',violation,'valid',all(isfinite([x;y]))&&violation<=1e-10,'distanceUpper',norm(x-y)+1e-8,'numericAllowance',1e-8);
end
