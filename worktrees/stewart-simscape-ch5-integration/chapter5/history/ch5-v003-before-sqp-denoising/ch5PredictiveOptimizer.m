function optimizer=ch5PredictiveOptimizer(r,active,polish)
% Multiple shooting is an implementation of the same control-increment
% decision: every auxiliary state is fixed by the actuator-augmented fd.
persistent caches
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
if nargin<3,polish=false;end
key=char(r.mode+"|"+r.c.version+"|"+mat2str(active)+"|"+string(polish));
if isKey(caches,key),optimizer=caches(key);return;end
import casadi.*
H=r.c.H;n=r.plant.n;ns=n;no=r.c.max_obstacles;
% Bound derivative lane width without changing any primal map. Smaller
% generated functions can be optimized by the native compiler efficiently.
xx=MX.sym('lane_x',n);uu=MX.sym('lane_u',6);
ff=r.plant.fd(xx,uu,zeros(6,1));
fd=Function('ch5_fd_lanes',{xx,uu},{ff},struct('max_num_dir',4));
mech=Function('ch5_mechanical_lanes',{xx,uu},{r.plant.mechanicalValues(xx,uu)},struct('max_num_dir',4));
D=MX.sym('D',6,H);U=MX.sym('U',6,H);X=MX.sym('X',ns,H);qa=MX.sym('qa',6);ua=MX.sym('ua',6);
P=MX.sym('P',n+6+6+22*no+6*H);x0=P(1:n);previous=P(n+(1:6));goal=P(n+6+(1:6));
O=reshape(P(n+12+(1:22*no)),22,no);prior=reshape(P(n+12+22*no+1:end),6,H);
z=[D(:);U(:);X(:);qa;ua];g={};lower=[];upper=[];gScale=[];last=previous;state=x0;cost=0;
ps=[.1;.1;.1;deg2rad(10)*ones(3,1)];vs=[.1;.1;.1;.2;.2;.2];
stateScale=[ps;vs;1000*ones(n-12,1)];
for j=1:H
    u=U(:,j);g{end+1}=u-last-D(:,j);lower=[lower;zeros(6,1)];upper=[upper;zeros(6,1)];last=u;
    gScale=[gScale;1000*ones(6,1)];
    next=fd(state,u);
    g{end+1}=X(:,j)-next(1:ns);lower=[lower;zeros(ns,1)];upper=[upper;zeros(ns,1)];
    % Tight shooting defects prevent small per-interval residuals from
    % accumulating into a terminal-velocity error in the true rollout.
    gScale=[gScale;.01*stateScale(1:ns)];
    g{end+1}=D(:,j);lower=[lower;-r.strict.forceRateLimit*r.c.Ts];upper=[upper;r.strict.forceRateLimit*r.c.Ts];
    gScale=[gScale;160*ones(6,1)];
    g{end+1}=u;lower=[lower;r.strict.forceMin];upper=[upper;r.strict.forceMax];
    gScale=[gScale;1000*ones(6,1)];
    % The nonlinear search enforces interval endpoints. Every internal point
    % is checked on the exact rollout before a candidate can be executed.
    % At an exactly feasible shooting solution these states are the same
    % endpoints as fd's trace. Avoid differentiating unused interior trace
    % outputs; the execution gate still checks the complete true rollout.
    for k=1:2
        if k==1,tx=state;offset=0;else,tx=X(:,j);offset=r.c.Ts;end
        values=mech(tx,u);ot=O;
        ot(1:3,:)=O(1:3,:)+((j-1)*r.c.Ts+offset)*O(16:18,:);
        gap=MX.zeros(numel(active),1);for oi=1:numel(active),gap(oi)=r.collision.single(tx(1:6),ot(:,active(oi)));end
        % Actual force equals bounded U in ideal mode, and is clipped to the
        % same bounds by the nonideal actuator map. Repeating those implied
        % inequalities at every inner point adds no restriction.
        value=[values(1:19);gap];g{end+1}=value;
        lower=[lower;r.physical.lmin;-r.physical.actuator.ldotMax;-r.physical.actuator.lddotMax;r.strict.sigmaSafe^2; ...
            ones(numel(active),1)*(r.c.safe_distance+r.c.sample_hold_margin)];
        upper=[upper;r.physical.lmax;r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;inf;inf(numel(active),1)];
        gScale=[gScale;ones(6,1);r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;r.strict.sigmaSafe^2;.1*ones(numel(active),1)];
    end
    poseError=(X(1:6,j)-qa)./ps;velocityError=X(7:12,j)./vs;
    offset=(X(1:6,j)-goal)./ps;delta=D(:,j)/160;priorError=(D(:,j)-prior(:,j))/160;
    effort=u/2000;
    cost=cost+r.c.Ts*(dot(poseError,poseError)+.1*dot(velocityError,velocityError)+.05*dot(offset,offset)+.01*dot(delta,delta)+.001*dot(effort,effort))+.05*dot(priorError,priorError);
    state=X(:,j);
end
stationary=r.ad.motion(qa,zeros(6,1),ua,zeros(6,1));
g{end+1}=stationary(7:12);lower=[lower;zeros(6,1)];upper=[upper;zeros(6,1)];
gScale=[gScale;ones(6,1)];
steady=[qa;MX.zeros(6,1)];if n>12,steady=[steady;repmat(ua,(n-12)/6,1)];end
terminalRows=numel(lower)+(1:n);
g{end+1}=state-steady;lower=[lower;zeros(n,1)];upper=[upper;zeros(n,1)];
gScale=[gScale;stateScale];
if n>12
    % Slew and one-step command memory both equal the final bounded input
    % after the ramp. Enforcing both terminal equalities duplicates rows.
    % Keep command memory as an equality and use the registered numerical
    % steady-state band for slew/lag. Full terminal actuator error is audited.
    bandRows=terminalRows(13:24);
    lower(bandRows)=-r.c.terminal_actuator_numerical_tolerance;
    upper(bandRows)=r.c.terminal_actuator_numerical_tolerance;
    gScale(bandRows)=r.c.terminal_actuator_numerical_tolerance;
end
g{end+1}=ua;lower=[lower;r.strict.forceMin];upper=[upper;r.strict.forceMax];
gScale=[gScale;1000*ones(6,1)];
% The artificial equilibrium must itself satisfy the mechanical bounds.
% Its first continuation input must respect the final preview input slew.
steadyValues=mech(steady,ua);
g{end+1}=steadyValues(1:19);
lower=[lower;r.physical.lmin;-r.physical.actuator.ldotMax;-r.physical.actuator.lddotMax;r.strict.sigmaSafe^2];
upper=[upper;r.physical.lmax;r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;inf];
gScale=[gScale;ones(6,1);r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;r.strict.sigmaSafe^2];
g{end+1}=ua-U(:,end);lower=[lower;-r.strict.forceRateLimit*r.c.Ts];upper=[upper;r.strict.forceRateLimit*r.c.Ts];
gScale=[gScale;160*ones(6,1)];
og=O;og(1:3,:)=O(1:3,:)+H*r.c.Ts*O(16:18,:);
terminalGap=MX.zeros(numel(active),1);for oi=1:numel(active),terminalGap(oi)=r.collision.single(qa,og(:,active(oi)));end
g{end+1}=terminalGap;lower=[lower;ones(numel(active),1)*(r.c.safe_distance+r.c.sample_hold_margin)];upper=[upper;inf(numel(active),1)];
gScale=[gScale;.1*ones(numel(active),1)];
steadyOffset=(qa-goal)./ps;cost=cost+100*dot(steadyOffset,steadyOffset);
constraints=vertcat(g{:});options=ch5JitOptions();options.print_time=false;
equalityTolerance=1e-5*gScale;
equalityTolerance(terminalRows)=[repmat(r.c.terminal_position_numerical_tolerance,3,1); ...
    repmat(deg2rad(r.c.terminal_rotation_numerical_tolerance_degrees),3,1); ...
    repmat(r.c.terminal_linear_speed_numerical_tolerance,3,1); ...
    repmat(deg2rad(r.c.terminal_angular_speed_numerical_tolerance_degrees),3,1); ...
    repmat(r.c.terminal_actuator_numerical_tolerance,n-12,1)];
options.ipopt=struct('print_level',0,'linear_solver','ma27','hessian_approximation','limited-memory', ...
    'max_iter',r.c.projection_iterations,'tol',1e-6,'acceptable_tol',1e-5,'warm_start_init_point','yes');
if polish,options.ipopt.max_iter=r.c.final_projection_iterations;end
options.calc_lam_p=false;
xScale=[160*ones(6*H,1);1000*ones(6*H,1);repmat(stateScale(1:ns),H,1);ps;1000*ones(6,1)];
y=MX.sym('scaled_decision',numel(z));
scaledCost=substitute(cost,z,xScale.*y);scaledConstraints=substitute(constraints,z,xScale.*y)./gScale;
optimizer=struct('solver',nlpsol('ch5_predictive_projection','ipopt',struct('x',y,'p',P,'f',scaledCost,'g',scaledConstraints),options), ...
    'check',Function('ch5_predictive_constraints',{z,P},{constraints,cost},ch5JitOptions()), ...
    'lower',lower,'upper',upper,'x_scale',xScale,'g_scale',gScale,'equality_tolerance',equalityTolerance,'terminal_rows',terminalRows,'n',n,'shooting_n',ns,'H',H, ...
    'state_offset',12*H,'parameter_dimension',numel(P),'decision_dimension',numel(z), ...
    'polish',polish,'input_trust_region',r.c.denoise_input_trust_region);
caches(key)=optimizer;
end
