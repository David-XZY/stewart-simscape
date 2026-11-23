function report=pgAudit(runName)
if nargin<1, runName='payload_audit_v1'; end
c=pgConfig(runName); base=bdMakeTask([],[0,0,0,7.5],c); m=base.model; checks={};
p=pgMakePayload('cylinder',[1.2,.15],100); t=pgFinalizeTask(base,p,c);
assert(norm(t.model.dynamics.inertiaAtCOM_P-m.dynamics.inertiaAtCOM_P,'fro')<1e-12);
assert(norm(t.derived.comP-m.dynamics.comP)<1e-12); checks{end+1}='nominal_composite_exact';
box=pgMakePayload('box',[1.2,.24,.20],100); assert(norm(box.inertiaCOM-diag([.813333333333333,12.3333333333333,12.48]),'fro')<1e-10); checks{end+1}='box_analytic_inertia';
assert(numel(t.context)==99 && ~isfield(t.scene,'objectCylinder')); [sig,meta]=pgPhysicsSignature(t); assert(numel(sig)==255&&all(isfinite(sig))); checks{end+1}='basic99_signature354';
heavy=pgFinalizeTask(base,pgMakePayload('cylinder',[1.2,.15],400),c);
assert(strcmp(pgProblemKey(t,c),pgProblemKey(heavy,c)));
bt=pgFinalizeTask(base,box,c); assert(~strcmp(pgProblemKey(t,c),pgProblemKey(bt,c))); checks{end+1}='numeric_mass_structural_shape_cache';
pk=pgPhysics(t.model); po=dpPhysics(m); rng(20260919,'twister');
Q=repmat([0;0;1.1;0;0;0],1,1000)+[.1;.1;.08;.12;.12;.15].*randn(6,1000); V=.08*randn(6,1000); A=.2*randn(6,1000);
[l,ld,ldd,f,j,w]=pk.point(Q,V,A); [ol,od,odd,of,oj,ow]=po.point(Q,V,A);
err=max(abs(full([l(:)-ol(:);ld(:)-od(:);ldd(:)-odd(:);f(:)-of(:);j(:)-oj(:);w(:)-ow(:)])));
assert(err<1e-8); checks{end+1}='1000_states_legacy_casadi_equivalence';
ph=pgPhysics(heavy.model); [lh,~,~,fh]=ph.point(Q,V,A); assert(max(abs(full(lh(:)-l(:))))<1e-12 && max(abs(full(fh(:)-f(:))))>1);
assert(isequal(pgClearance(t.q0,t),pgClearance(heavy.q0,heavy))); checks{end+1}='mass_changes_force_only';
for shape={'cylinder','box'}
    if strcmp(shape{1},'cylinder'), test=t; else, test=bt; end
    test.scene.hood.obstacles(1).center_S=test.q0(1:3)+[0;0;.15];
    wr=pgDistanceWitness(test.q0,test,1); assert(wr.valid&&wr.distanceUpper<.01);
    cr=pgCollision(test.q0,test,'hard'); assert(strcmp(cr.status{1},'witnessed_violation'));
end
checks{end+1}='two_shape_collision_membership_witness';
assert(c3SolverGate(true,'Solve_Succeeded',1e-6,1e-6,1e-6,true,true,true));
assert(~c3SolverGate(true,'Solve_Succeeded',0,0,1.1e-6,true,true,true));
assert(~c3SolverGate(true,'Solved_To_Acceptable_Level',0,0,0,true,true,true));
assert(~c3SolverGate(true,'Solve_Succeeded',NaN,0,0,true,true,true));
assert(~c3SolverGate(true,'Solve_Succeeded',0,0,0,true,false,true)); checks{end+1}='strict_solver_status_residual_bounds_budget';
assert(c.noiseSchedule.alphaBar(end)<1e-6 && c.noiseSchedule.posteriorVariance(1)==0); checks{end+1}='cosine_schedule_terminal';
% Oracle values and derivatives for an independently implemented Torch kernel.
oracle=struct('q',Q.','v',V.','a',A.','L',full(l).','Ld',full(ld).','Ldd',full(ldd).','F',full(f).', ...
    'J',full(j),'W',full(w).','robot',m,'payload',p,'task',t,'schedule',c.noiseSchedule);
pb=pgPhysics(bt.model); [~,~,~,bf]=pb.point(Q,V,A); oracle.boxTask=bt; oracle.boxForce=full(bf).';
oracle.heavyTask=heavy; oracle.heavyForce=full(fh).';
[B,D,DD]=bdBasis(linspace(0,1,33),1,16); oracle.basis=B; oracle.basisD=D; oracle.basisDD=DD;
oracle.cylinderGap=pgClearanceBatch(Q(:,1:30),t).'; oracle.boxGap=pgClearanceBatch(Q(:,1:30),bt).';
oracle.envelope=c.envelope; oracle.signature=sig; oracle.baseline=bdBaseline(t);
import casadi.*
state=SX.sym('state',18); [~,~,~,ff]=pk.parameterPoint(state(1:6),state(7:12),state(13:18),pk.parameters);
gradientFun=Function('pg_force_gradient',{state},{gradient(sum1(ff.^2)/1e6,state)});
oracle.forceGradient=full(gradientFun([Q(:,1:30);V(:,1:30);A(:,1:30)])).';
dpWriteJson(fullfile(c.output,'physics_oracle.json'),oracle);
report=struct('passed',true,'count',numel(checks),'checks',{checks},'legacyAbsoluteError',err,'signature',meta,'codeHash',c.codeHash,'scope','Stage A MATLAB tests; Torch parity is a separate required gate.');
dpWriteJson(fullfile(c.output,'matlab_audit.json'),report); fprintf('PG_AUDIT_PASSED %d\n',numel(checks));
end
