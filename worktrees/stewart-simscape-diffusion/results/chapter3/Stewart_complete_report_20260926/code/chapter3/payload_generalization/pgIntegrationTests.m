function report=pgIntegrationTests(runName)
if nargin<1, runName='payload_integration_v1'; end
c=pgConfig(runName); base=bdMakeTask([],[0,0,0,7.5],c); base.scene.hood.obstacles=base.scene.hood.obstacles([]); base.family='open'; base.split='dev_train';
t=pgFinalizeTask(base,pgMakePayload('box',[1,.24,.20],100),c); cp=bdBaseline(t); bundle=struct('kind','spline','cp',cp,'T',t.T); checks={};
[ok,r]=pgAccept(bundle,t,'physical',tic,20,c); assert(ok,r.reason); checks{end+1}='open_box_full_acceptance';
bad=bundle; bad.cp(1,5)=NaN; [ok,r]=pgAccept(bad,t,'physical',tic,20,c); assert(~ok&&strcmp(r.reason,'nonfinite')); checks{end+1}='nan_rejected';
wrong=bundle; wrong.T=3; [ok,r]=pgAccept(wrong,t,'physical',tic,20,c); assert(~ok&&strcmp(r.reason,'duration_mismatch')); checks{end+1}='duration_rejected';
h=struct('labelId','infrastructure_only','task',t,'bundle',bundle,'fitted',bundle);
a=struct('library',{{h,h}},'trainX',repmat(t.context(:).',2,1),'inputScale',ones(1,99),'inputMean',t.context(:).','distanceVariant','equal_blocks');
out=pgPlan(t,'retrieval',5,123,a,c); assert(out.passed&&out.candidateChecked==1&&strcmp(out.returnStage,'direct')); assert(out.otherSeconds>=-1e-6); checks{end+1}='retrieval_early_return_and_exclusive_time';
cc=c; cc.batchTotals=4; out=pgPlan(t,'retrieval',10,123,a,cc); assert(out.passed&&out.candidateGenerated==4&&out.candidateChecked==1); checks{end+1}='whole_batch_count';
out=pgPlan(t,'retrieval',0,123,a,c); assert(~out.passed&&~out.returnedWithinBudget); checks{end+1}='expired_return_rejected';
asset=struct('method','ddpm_basic','contextSchema',c.contextSchema,'controlCount',16,'outputRepresentation','absolute_free_controls_column_major', ...
    'inputMean',zeros(99,1),'inputScale',ones(99,1),'outputMean',reshape(cp(:,4:end-3),[],1),'outputScale',ones(60,1), ...
    'noiseSchedule',pgNoiseSchedule(100),'layers',struct('W',zeros(60,161),'b',zeros(60,1)));
pgModelCheck(asset,'ddpm_basic',c); a.model=asset; a.callContext=t.context;
one=pgCandidates('ddpm_basic',t,1,a,tic,20,c,123); many=pgCandidates('ddpm_basic',t,1:4,a,tic,20,c,123);
assert(isequal(one{1}.bundle.cp,many{1}.bundle.cp)); checks{end+1}='stable_candidate_prefix';
corrupt=asset; corrupt.contextSchema='legacy93'; rejected=false; try, pgModelCheck(corrupt,'ddpm_basic',c); catch, rejected=true; end; assert(rejected); checks{end+1}='legacy_model_rejected';
tm=struct('kind','path_time_map','path',bundle,'pathHash',dpHash(bundle),'T',t.T,'timeControls',[]);
q=pgQuality(bundle,t,c); qm=pgQuality(tm,t,c); assert(abs(q.positionLengthM-qm.positionLengthM)<1e-10); checks{end+1}='cross_representation_quality';
clock=tic; [retimed,log]=pgRetime(bundle,t,clock,20,c); assert(log.solverSuccess); assert(strcmp(retimed.pathHash,dpHash(bundle))); r=pgValidate(retimed,t,c); assert(r.passed,r.reason); checks{end+1}='retiming_path_hash_and_acceptance';
test=pgFinalizeTask(bdMakeTask([],[0,0,0,7.5],c),pgMakePayload('box',[1.2,.24,.20],100),c); free=reshape(bdBaseline(test),[],1); free=free(19:end-18);
rng(20260919); free=free+.015*randn(size(free)); [v,g,groups]=pgPenalty(free,test,c); direction=randn(size(free)); direction=direction/norm(direction); step=1e-6;
vp=pgPenalty(free+step*direction,test,c); vm=pgPenalty(free-step*direction,test,c); fd=(vp-vm)/(2*step);
assert(abs(fd-g.'*direction)/max([1e-6,abs(fd),abs(g.'*direction)])<1e-3); checks{end+1}='casadi_svd_penalty_gradient';
dpWriteJson(fullfile(c.output,'penalty_oracle.json'),struct('free',free,'task',test,'envelope',c.envelope,'value',v,'gradient',g,'groups',groups));
report=struct('passed',true,'checks',{checks},'count',numel(checks),'codeHash',c.codeHash,'scope','Infrastructure only; synthetic weights are not a trained model.');
dpWriteJson(fullfile(c.output,'integration.json'),report); fprintf('PG_INTEGRATION_PASSED %d\n',numel(checks));
end
