function [aa,bb,info]=ch5ReducedJacobianV66(nodes,U,goal,time,r,k)
% Engineering partial sensitivity update; true rollout/constraints stay fresh.
H=r.c.H;n=r.plant.n;scale=[.1*ones(3,1);deg2rad(10)*ones(3,1);.1*ones(3,1);.2*ones(3,1);160*ones(n-12,1)];
maximumAge=.3;if isfield(r.reduced,'sensitivity_maximum_age'),maximumAge=r.reduced.sensitivity_maximum_age;end
assert(maximumAge>=.1&&maximumAge<=.5,'ch5:SensitivityAge','Bound development sensitivity age.');
old=[];if isappdata(0,'ch5ReducedSensitivity'),old=getappdata(0,'ch5ReducedSensitivity');end
if isfield(r.reduced,'reuse_sensitivities')&&~r.reduced.reuse_sensitivities,old=[];end
use=false(1,H);aa=zeros(n,n*H);bb=zeros(n,6*H);built=time*ones(1,H);errors=inf(1,H);distances=errors;
if ~isempty(old)&&strcmp(old.mode,char(r.mode))&&isequal(size(old.U),size(U))&&norm(goal-old.goal)<1e-10
    shift=round((time-old.time)/r.c.Ts);
    if shift>0&&shift<H&&abs(time-old.time-shift*r.c.Ts)<1e-8
        for j=1:H-shift
            b=j+shift;dx=nodes(:,j)-old.nodes(:,b);du=U(:,j)-old.U(:,b);
            A=old.aa(:,n*(b-1)+(1:n));B=old.bb(:,6*(b-1)+(1:6));
            prediction=old.nodes(:,b+1)+A*dx+B*du;
            errors(j)=norm((nodes(:,j+1)-prediction)./scale,inf);distances(j)=norm(dx./scale,inf);
            use(j)=time-old.built(b)<=maximumAge+1e-8&&distances(j)<=.05&&norm(du/160,inf)<=.2&&errors(j)<=1e-4;
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
setappdata(0,'ch5ReducedSensitivity',struct('mode',char(r.mode),'time',time,'built',built,'nodes',nodes,'U',U, ...
    'aa',aa,'bb',bb,'goal',goal));
info=struct('reused_stages',nnz(use),'fresh_stages',numel(fresh),'derivative_seconds',toc(timer), ...
    'linear_prediction_errors',errors,'scaled_state_distances',distances,'maximum_age_seconds',max(time-built), ...
    'engineering_thresholds',struct('maximum_age',maximumAge,'state_distance',.05,'input_distance',.2,'primal_error',1e-4), ...
    'bounded_derivative_error_proven',false);
end
