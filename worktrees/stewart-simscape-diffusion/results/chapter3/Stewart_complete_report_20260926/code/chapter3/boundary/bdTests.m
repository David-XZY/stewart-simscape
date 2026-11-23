function result=bdTests()
c=bdConfig('boundary_tests_v1'); t=bdMakeTask([], [0,0,0,7.5],c); checks={};
assert(t.endpointPassed); checks{end+1}='endpoint_and_declared_envelopes';
u=linspace(0,1,103); [B,D,DD]=bdBasis(u,1,16); [Bo,Do,DDo]=dpBasis(u,1);
assert(max(abs([B(:)-Bo(:);D(:)-Do(:);DD(:)-DDo(:)]))<1e-10);
checks{end+1}='legacy_basis_equivalence';
b=struct('kind','spline','cp',bdBaseline(t,16),'T',t.T); f=bdPathFunction(b);
[Q,V,A]=f(u); tr=bdReconstruct(b,u*t.T);
assert(max(abs([full(Q(:))-tr.q(:);full(V(:))/t.T-tr.v(:);full(A(:))/t.T^2-tr.a(:)]))<1e-8);
checks{end+1}='symbolic_path_derivatives';
h=struct('bundle',b,'fitted',b,'task',t); target=bdMakeTask([],[.01,-.01,deg2rad(5),6],c);
for name={'A0','A5','A7','A5-C'}
    path=bdAdapt(h,target,name{1},c); f=bdPathFunction(path); [q,v,a]=f([0,1]);
    assert(max(abs([full(q(:,1))-target.q0;full(q(:,2))-target.qGoal;reshape(full(v),[],1);reshape(full(a),[],1)]))<1e-7);
end
checks{end+1}='four_adapters_rest_boundaries';
r=bdVariationBound([0,1,0],2,1,0); assert(abs(r.timeLowerBound-1.5)<1e-12);
r=bdVariationBound([0,1,0],2,1,.1); assert(r.timeLowerBound<1.5);
checks{end+1}='asymmetric_variation_and_error_allowance';
map=struct('kind','path_time_map','path',b,'pathHash',dpHash(b),'T',t.T,'timeControls',[]);
mapped=bdReconstruct(map,u*t.T); assert(max(abs(mapped.q(:)-tr.q(:)))<1e-10);
checks{end+1}='uniform_retime_geometry';
p=sgPhysics(t.model); s=.47; [q,qs,qss]=f(s); q=full(q); qs=full(qs); qss=full(qss);
[~,~,~,F0]=p.point(q,zeros(6,1),zeros(6,1)); [~,~,~,Fx]=p.point(q,qs,qss); [~,~,~,Fu]=p.point(q,zeros(6,1),qs);
beta=.3; alpha=-.2; [~,~,~,F]=p.point(q,qs*beta,qss*beta^2+qs*alpha);
assert(max(abs(full(F-F0-(Fx-F0)*beta^2-(Fu-F0)*alpha)))<1e-7);
checks{end+1}='force_chain_rule';
assert(c3SolverGate(true,'Solve_Succeeded',1e-6,1e-6,1e-6,true,true,true));
assert(~c3SolverGate(true,'Solve_Succeeded',0,0,2e-6,true,true,true));
assert(~c3SolverGate(true,'Solve_Succeeded',NaN,0,0,true,true,true));
assert(~c3SolverGate(false,'Maximum_Iterations_Exceeded',0,0,0,true,true,true));
checks{end+1}='three_part_solver_gate';
collision=t; collision.scene.hood.obstacles(1).center_S=t.q0(1:3)+[0;0;.15];
w=bdDistanceWitness(t.q0,collision,1); assert(w.valid&&w.distanceUpper<.01);
checks{end+1}='feasible_point_pair_upper_witness';
bad=b; bad.cp(1,5)=NaN; [ok,r]=bdAccept(bad,t,'physical',tic,10,c); assert(~ok&&strcmp(r.reason,'nonfinite'));
checks{end+1}='nonfinite_rejection';
wrong=b; wrong.T=t.T+1; [ok,r]=bdAccept(wrong,t,'physical',tic,10,c); assert(~ok&&strcmp(r.reason,'duration_mismatch'));
wrong=map; wrong.timeControls=[0,0,0,.8,.2,.5,.6,.7,.9,1,1,1]; [ok,r]=bdAccept(wrong,t,'physical',tic,10,c); assert(~ok&&strcmp(r.reason,'time_map_contract'));
checks{end+1}='fixed_duration_and_monotone_time_map';
changed=t; changed.T=4; changed.qGoal(2)=changed.qGoal(2)+.01; changed.scene.hood.obstacles(1).center_S(2)=.5;
assert(strcmp(bdProblemKey(t,c),bdProblemKey(changed,c)));
changed.model.dynamics.totalMass=151; assert(~strcmp(bdProblemKey(t,c),bdProblemKey(changed,c)));
changed=t; changed.model.actuator.forceMax(1)=1900; assert(~strcmp(bdProblemKey(t,c),bdProblemKey(changed,c)));
checks{end+1}='parameter_vs_physics_cache_keys';
result=struct('passed',true,'checks',{checks},'count',numel(checks),'codeHash',c.codeHash,'matlab',version);
dpWriteJson(fullfile(c.output,'tests.json'),result); fprintf('BD_TESTS_PASSED %d\n',numel(checks));
end
