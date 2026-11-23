function r=bdRobotEnvelope(task,Q,c)
% Certified separating-plane gaps for declared idealized bodies vs fixtures.
% Negative certificate means unknown, never a collision witness.
m=task.model; e=c.envelope; obs=task.scene.hood.obstacles;
gaps=zeros(8*numel(obs),size(Q,2));
for k=1:size(Q,2)
    R=rpy2rotmZYX(Q(4:6,k)); p=Q(1:3,k); P=p+R*m.B(:,m.legMap);
    for j=1:numel(obs)
        o=obs(j); dirs=[eye(3),-eye(3),o.R_S,-o.R_S];
        box=o.halfSize.'*abs(o.R_S.'*dirs);
        ax=R(:,3).'*dirs;
        gaps((j-1)*8+1,k)=max((o.center_S-p).'*dirs-box-e.platformHalfHeight*abs(ax)-e.platformRadius*sqrt(max(0,1-ax.^2)));
        ax=dirs(3,:);
        gaps((j-1)*8+2,k)=max((o.center_S-e.baseCenter).'*dirs-box-e.baseHalfHeight*abs(ax)-e.baseRadius*sqrt(max(0,1-ax.^2)));
        for i=1:6
            center=(m.A(:,i)+P(:,i))/2; half=(P(:,i)-m.A(:,i))/2;
            gaps((j-1)*8+2+i,k)=max((o.center_S-center).'*dirs-box-abs(half.'*dirs)-e.legRadius);
        end
    end
end
r=struct('passed',all(isfinite(gaps(:)))&&all(gaps(:)>=.01-1e-8), ...
    'minCertificate',min(gaps(:)),'gaps',gaps,'scope',e.scope);
end
