function [aa,bb,info]=ch5ReducedJacobianV26(nodes,U,goal,time,r,k)
% Primal curvature screening and actuator branch checks; no error-bound theorem claimed.
H=r.c.H;n=r.plant.n;scale=[.1*ones(3,1);deg2rad(10)*ones(3,1);.1*ones(3,1);.2*ones(3,1);160*ones(n-12,1)];
maximumAge=option(r,'sensitivity_maximum_age',.10000001);relativeBound=option(r,'sensitivity_relative_error',.001);stateBound=option(r,'sensitivity_state_distance',.005);inputBound=option(r,'sensitivity_input_distance',.02);
old=[];if isappdata(0,'ch5ReducedSensitivityV26'),old=getappdata(0,'ch5ReducedSensitivityV26');end
if isfield(r.reduced,'reuse_sensitivities')&&~r.reduced.reuse_sensitivities,old=[];end
use=false(1,H);aa=zeros(n,n*H);bb=zeros(n,6*H);built=time*ones(1,H);errors=inf(1,H);distances=errors;curvature=errors;branchChanged=false(1,H);
signature=branches(nodes(:,1:H),U,r);
if ~isempty(old)&&strcmp(old.mode,char(r.mode))&&isequal(size(old.U),size(U))
    shift=round((time-old.time)/r.c.Ts);
    if shift>=0&&shift<H&&abs(time-old.time-shift*r.c.Ts)<1e-8
        for j=1:H-shift
            b=j+shift;dx=nodes(:,j)-old.nodes(:,b);du=U(:,j)-old.U(:,b);
            A=old.aa(:,n*(b-1)+(1:n));B=old.bb(:,6*(b-1)+(1:6));change=A*dx+B*du;
            errors(j)=norm((nodes(:,j+1)-old.nodes(:,b+1)-change)./scale,inf);distances(j)=norm(dx./scale,inf);
            curvature(j)=errors(j)/max(1e-12,norm(change./scale,inf));branchChanged(j)=any(signature(:,j)~=old.branches(:,b));
            use(j)=time-old.built(b)<=maximumAge&&distances(j)<=stateBound&&norm(du/160,inf)<=inputBound&&errors(j)<=1e-5&&curvature(j)<=relativeBound&&~branchChanged(j);
            if use(j),aa(:,n*(j-1)+(1:n))=A;bb(:,6*(j-1)+(1:6))=B;built(j)=old.built(b);end
        end
    end
end
fresh=find(~use);timer=tic;
if ~isempty(fresh)
    mapped=k.stage.map(numel(fresh));[~,A,B]=mapped(nodes(:,fresh),U(:,fresh));A=full(A);B=full(B);
    for j=1:numel(fresh)
        index=fresh(j);aa(:,n*(index-1)+(1:n))=A(:,n*(j-1)+(1:n));bb(:,6*(index-1)+(1:6))=B(:,6*(j-1)+(1:6));
    end
end
setappdata(0,'ch5ReducedSensitivityV26',struct('mode',char(r.mode),'time',time,'built',built,'nodes',nodes,'U',U,'aa',aa,'bb',bb,'branches',signature));
info=struct('reused_stages',nnz(use),'fresh_stages',numel(fresh),'derivative_seconds',toc(timer),'linear_prediction_errors',errors, ...
    'scaled_state_distances',distances,'primal_curvature',curvature,'actuator_branch_changes',nnz(branchChanged),'maximum_age_seconds',max(time-built), ...
    'engineering_thresholds',struct('maximum_age',maximumAge,'state_distance',stateBound,'input_distance',inputBound,'primal_error',1e-5,'relative_error',relativeBound), ...
    'bounded_derivative_error_proven',false);
end
function signature=branches(nodes,U,r)
if r.plant.n==12,signature=false(1,size(U,2));return;end
p=r.plant.profile;h=p.delay-r.c.Ts*floor(p.delay/r.c.Ts);rate=p.slewRate;
first=nodes(25:30,:);if p.deadzone>0,first=sign(first).*max(abs(first)-p.deadzone,0);end
target=U;if p.deadzone>0,target=sign(target).*max(abs(target)-p.deadzone,0);end
delta=first-nodes(13:18,:);slew=nodes(13:18,:)+min(max(delta,-rate*h),rate*h);second=target-slew;
signature=[delta>=0;second>=0;nodes(19:24,:)<=p.forceMin;nodes(19:24,:)>=p.forceMax];
for segment=1:2
    duration=h;d=delta;if segment==2,duration=r.c.Ts-h;d=second;end
    steps=max(1,ceil(duration/(r.c.Ts/r.plant.substeps)));
    for point=1:2*steps
        at=duration*point/(2*steps);signature=[signature;abs(d)>rate*at]; %#ok<AGROW>
    end
end
end
function value=option(r,name,default)
value=default;if isfield(r.reduced,name),value=r.reduced.(name);end
end
