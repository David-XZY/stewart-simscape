function gaps=bdEnvelopeSymbolic(q,o,m,e,p)
% Same finite separating planes as bdRobotEnvelope, usable inside CasADi NLP.
import casadi.*
R=p.pose(q); P=q(1:3)+R*m.B(:,m.legMap); dirs=MX([eye(3),-eye(3),o.R_S,-o.R_S]);
box=o.halfSize.'*abs(o.R_S.'*dirs); ax=R(:,3).'*dirs;
gaps=mmax((o.center_S-q(1:3)).'*dirs-box-e.platformHalfHeight*abs(ax)-e.platformRadius*sqrt(fmax(0,1-ax.^2)+e.supportSmoothingSquared));
ax=dirs(3,:);
gaps=[gaps;mmax((o.center_S-e.baseCenter).'*dirs-box-e.baseHalfHeight*abs(ax)-e.baseRadius*sqrt(fmax(0,1-ax.^2)+e.supportSmoothingSquared))];
for i=1:6
    center=(m.A(:,i)+P(:,i))/2; half=(P(:,i)-m.A(:,i))/2;
    gaps=[gaps;mmax((o.center_S-center).'*dirs-box-abs(half.'*dirs)-e.legRadius)];
end
end

