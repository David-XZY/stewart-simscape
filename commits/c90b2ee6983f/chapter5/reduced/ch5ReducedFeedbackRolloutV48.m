function fun=ch5ReducedFeedbackRolloutV48(r)
persistent cached keySaved
key=char(r.mode+"|"+r.c.version+"|block10");
if ~isempty(keySaved)&&strcmp(key,keySaved),fun=cached;return;end
import casadi.*
H=r.c.H;n=r.plant.n;x=MX.sym('feedback_initial',n);warm=MX.sym('feedback_nodes',n,H+1);
base=MX.sym('feedback_base',6,H);K=MX.sym('feedback_gain',6,n);D=MX.sym('feedback_increment',6,H/10);
state=x;nodes=x;U=MX.zeros(6,H);correction=MX.zeros(6,1);
for j=1:H
    if mod(j-1,10)==0,correction=correction+160*D(:,ceil(j/10));end
    u=base(:,j)+correction-K*(state-warm(:,j));U(:,j)=u;
    state=r.plant.fd(state,u,MX.zeros(6,1));nodes=[nodes,state]; %#ok<AGROW>
end
fun=Function('ch5_feedback_condensed_rollout_v48',{x,warm,base,K,D},{U,nodes},ch5ProjectionJitOptions());
cached=fun;keySaved=key;
end
