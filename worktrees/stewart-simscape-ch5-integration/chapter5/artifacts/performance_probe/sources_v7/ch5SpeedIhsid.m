function plan=ch5SpeedIhsid(q0,v0,goal,o,runtime,duration,time)
% Implicit Hermite-Simpson direct collocation, with four registered starts.
% No insertion equality or waypoint equality is imposed on general tasks.
persistent caches warmLanes warmTime
if isempty(caches),caches=containers.Map('KeyType','char','ValueType','any');end
active=find(o(22,:)>.5);
if isempty(warmTime)||time<=warmTime,warmLanes=cell(4,1);warmTime=-inf;end
% A fixed boundary configuration that intersects a predicted obstacle cannot
% satisfy this collocation problem. Reject all four starts with the same
% geometric witness, rather than spending 1000 iterations on each one.
precheck=tic;endObstacles=o;endObstacles(1:3,:)=o(1:3,:)+duration*o(16:18,:);
for boundary=1:2
    if boundary==1,pose=q0;obstacles=o;else,pose=goal;obstacles=endObstacles;end
    gaps=full(runtime.collision.value(pose,obstacles));
    for index=active
        [distance,~,verified]=ch5GjkDistance(pose,obstacles(:,index),runtime.physical,runtime.scene);
        % A violated fixed-endpoint certificate is already a violated NLP
        % row, independently of whether GJK converges. Keep a 1 mm witness
        % margin relative to the much smaller numerical acceptance error.
        certificateInfeasible=gaps(index)<runtime.c.safe_distance+runtime.c.sample_hold_margin-.001;
        if certificateInfeasible||(verified&&distance<runtime.c.safe_distance-.001)
            attempts=cell(4,1);
            for start=1:4,attempts{start}=struct('start',start,'status','fixed_boundary_infeasible', ...
                'solver_success',false,'solver_called',false,'boundary',boundary,'obstacle',index, ...
                'fixed_constraint_certificate',gaps(index),'certificate_threshold',runtime.c.safe_distance+runtime.c.sample_hold_margin, ...
                'distance_upper_bound',distance,'distance_verified',verified,'seconds',0);end
            plan=struct('accepted',false,'attempts',{attempts},'seconds',toc(precheck), ...
                'initial_state',[q0;v0],'goal',goal,'precheck_infeasible',true);return;
        end
    end
end
key=sprintf('%s|%s',runtime.c.version,mat2str(active));
buildTimer=tic;if ~isKey(caches,key),caches(key)=build(runtime,active);end
buildSeconds=toc(buildTimer);
cache=caches(key);
N=60;h=duration/N;attempts=cell(4,1);best=inf;plan=struct('accepted',false,'attempts',[]);
parameter=[q0;v0;goal;o(:);duration];total=tic;
for start=1:4
    fprintf('IHSID start %d of 4, duration %.3fs\n',start,duration);
    startTimer=tic;Q=zeros(6,N+1);V=Q;A=Q;F=Q;Am=zeros(6,N);Fm=Am;
    axis=zeros(6,1);if start>1,axis(mod(start-2,3)+1)=.12;end
    if isempty(warmLanes{start})||~isfinite(warmTime)
    for k=1:N+1
        s=(k-1)/N;t=s*duration;
        blend=10*s^3-15*s^4+6*s^5;bd=(30*s^2-60*s^3+30*s^4)/duration;bdd=(60*s-180*s^2+120*s^3)/duration^2;
        velocityBlend=t*(1-s)^3;
        Q(:,k)=q0+(goal-q0)*blend+v0*velocityBlend+axis*sin(pi*s)^2;
        V(:,k)=(goal-q0)*bd+v0*(1-s)^2*(1-4*s)+axis*pi/duration*sin(2*pi*s);
        A(:,k)=(goal-q0)*bdd+v0*(-6+18*s-12*s^2)/duration+axis*2*pi^2/duration^2*cos(2*pi*s);
        [d,G]=runtime.ad.dynamics(Q(:,k),V(:,k));F(:,k)=full(G)\(A(:,k)-full(d));
    end
    for k=1:N
        xm=([Q(:,k);V(:,k)]+[Q(:,k+1);V(:,k+1)])/2+h/8*([V(:,k);A(:,k)]-[V(:,k+1);A(:,k+1)]);
        Am(:,k)=(A(:,k)+A(:,k+1))/2;
        [d,G]=runtime.ad.dynamics(xm(1:6),xm(7:12));Fm(:,k)=full(G)\(Am(:,k)-full(d));
    end
    end
    initialSource="cold";
    if ~isempty(warmLanes{start})&&isfinite(warmTime)
        [Q,V,A,F,Am,Fm]=ch5SpeedIhsidGuess(warmLanes{start},time-warmLanes{start}.warm_time,q0,v0,goal,duration,runtime);initialSource="shifted_previous_lane";
    end
    z0=[Q(:);V(:);A(:);F(:);Am(:);Fm(:)];
    guessSeconds=toc(startTimer);solverTimer=tic;
    try
        sol=cache.solver('x0',z0,'p',parameter,'lbg',cache.lower,'ubg',cache.upper);
        solverSeconds=toc(solverTimer);stats=cache.solver.stats();z=full(sol.x);g=full(cache.constraints(z,parameter));
        violation=max([cache.lower-g;g-cache.upper;0]);obj=full(sol.f);
        solution=unpack(z,duration,N);
        denseTimer=tic;
        if stats.success&&violation<=1e-5
            dense=ch5SpeedPlanPostcheck(solution,o,runtime);
        else
            dense=struct("accepted",false,"minimum_margin",nan,"maximum_violation",nan,"samples",0,"skipped_rejected_solver",true);
        end
        denseSeconds=toc(denseTimer);
        accepted=stats.success&&violation<=1e-5&&dense.accepted;
        attempts{start}=struct('start',start,'status',stats.return_status,'solver_success',stats.success, ...
            'constraint_violation',violation,'postcheck',dense,'objective',obj,'seconds',toc(startTimer));
        attempts{start}.initial_source=initialSource;attempts{start}.solver_seconds=solverSeconds;attempts{start}.guess_seconds=guessSeconds;attempts{start}.dense_seconds=denseSeconds;attempts{start}.iterations=stats.iter_count;
        if accepted,solution.warm_time=time;warmLanes{start}=solution;end
        fprintf('IHSID start %d: %s, accepted=%d, violation=%.3g, dense=%.3g, %.3fs\n',start,stats.return_status,accepted,violation,dense.maximum_violation,toc(startTimer));
        if accepted&&obj<best,plan=solution;plan.accepted=true;best=obj;end
    catch ex
        attempts{start}=struct('start',start,'status','exception','identifier',ex.identifier,'message',ex.message,'seconds',toc(startTimer));
    end
end
warmTime=time;plan.build_seconds=buildSeconds;plan.attempts=attempts;plan.seconds=toc(total);plan.initial_state=[q0;v0];plan.goal=goal;
[centers,times]=cache.obstaclePrediction(parameter);
plan.obstacle_prediction_centers=full(centers);plan.obstacle_prediction_times=full(times);
end
function cache=build(r,active)
import casadi.*
N=60;Q=MX.sym('Q',6,N+1);V=MX.sym('V',6,N+1);A=MX.sym('A',6,N+1);F=MX.sym('F',6,N+1);
Am=MX.sym('Am',6,N);Fm=MX.sym('Fm',6,N);P=MX.sym('P',19+22*r.c.max_obstacles);
T=P(end);h=T/N;O=reshape(P(19:end-1),22,r.c.max_obstacles);z=[Q(:);V(:);A(:);F(:);Am(:);Fm(:)];
g={Q(:,1)-P(1:6),V(:,1)-P(7:12),Q(:,end)-P(13:18),V(:,end)};lb=zeros(24,1);ub=lb;
cost=0;predictionCenters={};predictionTimes={};
for k=1:N
    X0=[Q(:,k);V(:,k)];X1=[Q(:,k+1);V(:,k+1)];f0=[V(:,k);A(:,k)];f1=[V(:,k+1);A(:,k+1)];
    Xm=(X0+X1)/2+h/8*(f0-f1);fm=[Xm(7:12);Am(:,k)];
    g{end+1}=X1-X0-h/6*(f0+4*fm+f1);lb=[lb;zeros(12,1)];ub=[ub;zeros(12,1)];
    positions={X0,Xm};accelerations={A(:,k),Am(:,k)};forces={F(:,k),Fm(:,k)};times={(k-1)*h,(k-.5)*h};weights=[1+double(k>1),4];
    if k==N,positions{3}=X1;accelerations{3}=A(:,end);forces{3}=F(:,end);times{3}=T;weights(3)=1;end
    for j=1:numel(positions)
        x=positions{j};a=accelerations{j};u=forces{j};
        [equilibrium,mechanical]=r.ad.implicit(x(1:6),x(7:12),a,u);
        g{end+1}=equilibrium;lb=[lb;zeros(6,1)];ub=[ub;zeros(6,1)];
        ot=MX(O);ot(1:3,:)=O(1:3,:)+times{j}*O(16:18,:);
        predictionCenters{end+1}=ot(1:3,:);predictionTimes{end+1}=times{j};
        sigma=mechanical(19);gap=MX.zeros(numel(active),1);
        for oi=1:numel(active),gap(oi)=r.collision.single(x(1:6),ot(:,active(oi)));end
        values=[mechanical(1:19);gap;u];g{end+1}=values;
        lb=[lb;r.physical.lmin;-r.physical.actuator.ldotMax;-r.physical.actuator.lddotMax;r.strict.sigmaSafe^2;repmat(r.c.safe_distance+r.c.sample_hold_margin,numel(active),1);r.strict.forceMin];
        ub=[ub;r.physical.lmax;r.physical.actuator.ldotMax;r.physical.actuator.lddotMax;inf;inf(numel(active),1);r.strict.forceMax];
        s=times{j}/T;nominal=P(1:6)+(P(13:18)-P(1:6))*(10*s^3-15*s^4+6*s^5);
        dev=(x(1:6)-nominal)./[.1;.1;.1;deg2rad(10)*ones(3,1)];
        legacc=mechanical(13:18);
        cost=cost+h/6*weights(j)*(.1*dot(dev,dev)+.3*dot(legacc,legacc)/1.2^2+.4*.01^2/(sigma+1e-6));
    end
    rate=(F(:,k+1)-F(:,k))/h;g{end+1}=rate;lb=[lb;-r.strict.forceRateLimit];ub=[ub;r.strict.forceRateLimit];
    cost=cost+.08*h*dot(rate,rate)/1000^2;
end
options=ch5JitOptions();options.print_time=false;options.ipopt=struct('print_level',0,'linear_solver','ma27', ...
    'hessian_approximation','limited-memory','max_iter',1000,'tol',1e-7,'acceptable_tol',1e-6);
constraint=vertcat(g{:});cache=struct('solver',nlpsol('ch5_ihsid','ipopt',struct('x',z,'p',P,'f',cost,'g',constraint),options), ...
    'constraints',Function('ch5_ihsid_constraints',{z,P},{constraint}),'lower',lb,'upper',ub, ...
    'obstaclePrediction',Function('ch5_ihsid_obstacle_prediction',{P},{horzcat(predictionCenters{:}),horzcat(predictionTimes{:})}));
end
function p=unpack(z,T,N)
n=6*(N+1);p=struct('t',linspace(0,T,N+1),'q',reshape(z(1:n),6,N+1), ...
    'qd',reshape(z(n+(1:n)),6,N+1),'qdd',reshape(z(2*n+(1:n)),6,N+1),'F',reshape(z(3*n+(1:n)),6,N+1));
end
