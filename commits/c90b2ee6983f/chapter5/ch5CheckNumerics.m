function report=ch5CheckNumerics()
% Independent analytic geometry cases, rigid-frame invariance, and support
% certificate checks. These are development checks, never teacher labels.
[c,root]=ch5Init();[m,s]=ch5Physics(c);collision=ch5CollisionModel(m,s,c);
report=struct('analytic_distance_error',0,'certificate_excess',0,'cases',0);
for k=1:20
    angles=[.2*sin(k);.2*cos(k);.1*k];q=[.02*k;-.01*k;1.1;angles];R=ch5Rotation(angles);
    for offset=[1,.7,.65]
        o=[q(1:3)+R*[offset;0;.15];.1;.4;.4;R(:);zeros(6,1);1];
        [distance,contact,converged]=ch5GjkDistance(q,o,m,s);
        expected=max(offset-.1-s.objectCylinder.length/2,0);
        report.analytic_distance_error=max(report.analytic_distance_error,abs(distance-expected));
        report.certificate_excess=max(report.certificate_excess,full(collision.single(q,o))-distance);
        assert(converged,'ch5:GeometryConvergence','GJK did not converge on an analytic convex case.');
        assert(contact==(expected==0),'ch5:GeometryContact','Contact classification mismatch.');
        report.cases=report.cases+1;
    end
end
assert(report.analytic_distance_error<1e-5&&report.certificate_excess<=1e-6);
% Hidden geometry must not affect either controller's condition vector.
task=struct('goal_times',0,'goals',m.qHome,'obstacles',zeros(22,c.max_obstacles), ...
    'motion_velocity',zeros(3,c.max_obstacles),'turn_rate',zeros(1,c.max_obstacles),'visible_times',ones(1,c.max_obstacles)*3);
task.obstacles(7:15,:)=repmat(reshape(eye(3),9,1),1,c.max_obstacles);task.obstacles(22,1)=1;
other=task;other.obstacles(1:6,1)=[1;2;3;.1;.2;.3];other.motion_velocity(:,1)=[.005;.01;.015];
[g,a]=ch5Observe(task,2);[gg,b]=ch5Observe(other,2);
report.hidden_obstacle_condition_error=norm(ch5Context(zeros(12,1),zeros(6,1),g,a,"ideal-force")- ...
    ch5Context(zeros(12,1),zeros(6,1),gg,b,"ideal-force"),inf);
assert(report.hidden_obstacle_condition_error==0&&~any(vecnorm(a(16:18,:))));
[~,revealed]=ch5Observe(other,3);assert(revealed(22,1)==1&&norm(revealed(16:18,1)-other.motion_velocity(:,1))<1e-12);
% Validate normalized increment accumulation independent of any learned model.
previous=(250:255)';D=reshape(sin(1:360),6,60);U=previous+cumsum(D,2);
report.increment_roundtrip_error=norm(diff([previous,U],1,2)-D,'inf');assert(report.increment_roundtrip_error<1e-10);
report.passed=true;save(fullfile(root,'chapter5','artifacts','validation','numerics.mat'),'report');
fid=fopen(fullfile(root,'chapter5','artifacts','validation','numerics.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report));fclose(fid);
fprintf('Independent numerical checks passed: %d geometry cases.\n',report.cases);
end
