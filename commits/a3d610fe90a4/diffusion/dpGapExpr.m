function gap=dpGapExpr(q,n,o,task,p)
R=p.pose(q); axis=R*task.scene.objectCylinder.axis_P;
center=q(1:3)+R*task.scene.objectCylinder.center_P;
axial=n.'*axis; radial=n-axis*axial; e=1e-20;
gap=n.'*(o.center_S-center)-o.halfSize.'*sqrt((o.R_S.'*n).^2+e)- ...
    .5*task.scene.objectCylinder.length*sqrt(axial^2+e)- ...
    task.scene.objectCylinder.radius*sqrt(radial.'*radial+e);
end
