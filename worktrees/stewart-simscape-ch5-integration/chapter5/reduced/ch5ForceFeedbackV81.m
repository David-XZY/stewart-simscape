function rollout=ch5ForceFeedbackV81(r)
% Same block corrections and state feedback as V50, evaluated with mapaccum.
persistent cache
if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
key=char(r.mode+"|"+r.c.version);
if isKey(cache,key),rollout=cache(key);return;end
import casadi.*
H=r.c.H;n=r.plant.n;x=MX.sym('proposal_state',n);warm=MX.sym('proposal_warm',n);
base=MX.sym('proposal_base',6);K=MX.sym('proposal_feedback',6,n);correction=MX.sym('proposal_correction',6);
u=base+correction-K*(x-warm);next=r.plant.fd(x,u,MX.zeros(6,1));
stage=Function('ch5_proposal_scan_stage',{x,warm,base,K,correction},{next,u});scan=stage.mapaccum(H);
nodes=MX.sym('proposal_nodes',n,H+1);U=MX.sym('proposal_inputs',6,H);D=MX.sym('proposal_blocks',6,8);
starts=[1,6,16,26,36,46,51,56];block=arrayfun(@(j)find(starts<=j,1,'last'),1:H);
cumulative=MX.zeros(6,8);value=MX.zeros(6,1);
for j=1:8,value=value+160*D(:,j);cumulative(:,j)=value;end
[X,commands]=scan(x,nodes(:,1:H),U,repmat(K,1,H),cumulative(:,block));
rollout=Function('ch5_proposal_scan',{x,nodes,U,K,D},{commands,[x,X]},ch5ProjectionJitOptions());cache(key)=rollout;
end
