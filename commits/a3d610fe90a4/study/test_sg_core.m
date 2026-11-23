function test_sg_core()
c=sgSetup(); task=makeTask('simple',20262041,'development',false);
nominal=sgModel(task.model,task.scene,struct());
assert(abs(nominal.dynamics.totalMass-task.model.dynamics.totalMass)<1e-12);
assert(max(abs(nominal.dynamics.comP-task.model.dynamics.comP))<1e-12);
assert(max(abs(nominal.dynamics.inertiaAtCOM_P(:)-task.model.dynamics.inertiaAtCOM_P(:)))<1e-10);
p=sgPhysics(task.model); q=repmat(task.q0,1,11)+.01*reshape(sin(1:66),6,11);
[batch,~]=sgClearanceBatch(q,task);
for k=1:size(q,2), legacy=dpClearance(q(:,k),task,false); assert(max(abs(batch(:,k)-legacy(:)))<1e-10); end
asset=sgAssets(c); [a,~]=sgSample(task,1,987,asset); [b,~]=sgSample(task,4,987,asset);
assert(max(abs(a(:)-reshape(b(:,:,1),[],1)))<1e-10,'Candidate prefixes differ.');
files=dir(fullfile(c.source,'benchmark','*','diffusion','benchmark.mat')); compared=0;
for i=[1,2,11,12]
    s=load(fullfile(files(i).folder,files(i).name));
    for k=[1,4]
        bundle=struct('kind','spline','cp',s.candidates(:,:,k),'T',s.task.T);
        fast=sgValidate(bundle,s.task,c); gold=sgAudit(bundle,s.task,c);
        assert(fast.passed==gold.passed,'Fast/audit verdict differs.');
        if fast.passed
            assert(isequal(fast.time,gold.time));
            assert(max(abs(fast.clearance(:)-gold.clearance(:)))<1e-8);
        end
        compared=compared+1;
    end
end
% Parameter limits affect both evaluators; no legacy 2000 N shortcut.
static=struct('kind','spline','cp',repmat(task.q0,1,16),'T',task.T);
task.qGoal=task.q0; task.model.actuator.forceMax(:)=1; task.model.actuator.forceMin(:)=-1;
fast=sgValidate(static,task,c); gold=sgAudit(static,task,c); assert(~fast.passed && ~gold.passed);
bad=static; bad.cp(1,5)=NaN; fast=sgValidate(bad,task,c); assert(~fast.passed);
report=struct('passed',true,'comparedCandidates',compared,'nominalCompositeMatches',true,'commonNoisePrefixes',true);
dpWriteJson(fullfile(c.output,'core_tests.json'),report); fprintf('SG_CORE_PASSED %d candidate cross-checks\n',compared);
end
