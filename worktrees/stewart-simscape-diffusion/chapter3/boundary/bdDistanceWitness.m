function w=bdDistanceWitness(q,task,j)
% Alternating projections give feasible point pairs, NOT an optimality claim.
o=task.scene.hood.obstacles(j); R=rpy2rotmZYX(q(4:6)); cy=task.scene.objectCylinder;
center=q(1:3)+R*cy.center_P; axis=R*cy.axis_P;
x=center; y=o.center_S;
for k=1:80
    local=o.R_S.'*(x-o.center_S); y=o.center_S+o.R_S*max(-o.halfSize,min(o.halfSize,local));
    d=y-center; axial=max(-cy.length/2,min(cy.length/2,axis.'*d)); radial=d-axis*(axis.'*d);
    radial=radial*min(1,cy.radius/max(norm(radial),realmin)); x=center+axis*axial+radial;
end
dx=x-center; ly=o.R_S.'*(y-o.center_S); membership=max([abs(axis.'*dx)-cy.length/2;norm(dx-axis*(axis.'*dx))-cy.radius;abs(ly)-o.halfSize]);
% Floating point membership allowance and distance allowance explicitly saved.
valid=all(isfinite([x;y])) && membership<=1e-10;
w=struct('pointCylinder',x,'pointBox',y,'membershipViolation',membership, ...
    'distanceUpper',norm(x-y)+1e-8,'valid',valid,'numericAllowance',1e-8);
end
