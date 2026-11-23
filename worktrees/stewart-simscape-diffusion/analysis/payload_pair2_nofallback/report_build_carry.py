"""Generate a scheduling-only batch variant; the online planner remains byte-identical."""
from pathlib import Path
H=Path(__file__).resolve().parent
s=(H/'p2Batch.m').read_text(encoding='utf-8').replace('function p2Batch(requestFile)','function p3CarryBatch(requestFile)')
s=s.replace('jobs=r.jobs;', '''jobs=r.jobs;
jobMap=containers.Map('KeyType','char','ValueType','any');
for jj=1:numel(jobs), jobMap(key(jobs(jj),jobs(jj).B))=jobs(jj); end
oldRows=jsondecode(fileread(r.ihsidBaselineFile)); if ~iscell(oldRows), oldRows=num2cell(oldRows); end
oldMap=containers.Map('KeyType','char','ValueType','any');
for jj=1:numel(oldRows), row=oldRows{jj}; oldMap(key(row.job,row.B))=row; end
if ~isfolder(r.viewOutput), mkdir(r.viewOutput); end''')
s=s.replace(" if isfile(committed)\n  old=jsondecode(fileread(committed)); assert(strcmp(old.configHash,r.configHash)&&isfile(fullfile(path,'trajectory.mat')),'P2:Resume','Identity/artifact mismatch.'); continue;\n end", ''' [source,origin]=availableSource(job);
 if ~isempty(source)
  recordView(source,origin); heartbeat('resolved_existing'); continue;
 end
 assert(~isfield(r,'expectedNoNewMeasurements')||~r.expectedNoNewMeasurements,'P3:TestUnexpectedMeasurement','Carry test attempted new planning.');
 assert(~strcmp(job.method,'ihsid')||job.B==20,'P3:IHSIDScope','Only missing IHSID 20s calls may execute.');''')
s=s.replace(" heartbeat('committed'); fprintf", " recordView(compact,'new_measurement'); heartbeat('committed'); fprintf")
s=s.replace(' function heartbeat(stage)', ''' function kkey=key(j,b)
  kkey=sprintf('%s|%s|%d|%g',j.spec.taskId,j.method,j.repeat,b);
 end
 function [source,origin]=availableSource(j)
  source=[]; origin=''; same=[]; sameOrigin='';
  for b=[1,2,5,10,20]
   if b>j.B, break; end
   kk=key(j,b); candidate=[]; kind='';
   if strcmp(j.method,'ihsid')&&b<20
    assert(isKey(oldMap,kk),'P3:MissingBaseline','Missing existing IHSID budget.');
    candidate=oldMap(kk); kind='historical_ihsid';
   elseif isKey(jobMap,kk)
    priorJob=jobMap(kk); f=fullfile(r.output,'calls',priorJob.runId,'result.json');
    if isfile(f)
     candidate=jsondecode(fileread(f)); kind='existing_pair2';
     assert(strcmp(candidate.configHash,r.configHash)&&isfile(fullfile(r.output,'calls',priorJob.runId,'trajectory.mat')),'P3:Identity','Existing call differs.');
    end
   end
   if isempty(candidate), continue; end
   assert(strcmp(candidate.taskId,j.spec.taskId)&&strcmp(candidate.method,j.method)&&candidate.repeat==j.repeat&&candidate.trainingSeed==j.trainingSeed&&candidate.samplingSeed==j.samplingSeed,'P3:CrossBlock','Cannot inherit across tasks/methods/seeds.');
   if b==j.B, same=candidate; sameOrigin=kind; end
   if candidate.finalQualified
    assert(candidate.onlinePassed&&candidate.independentAuditPassed&&candidate.returnedWithinBudget&&candidate.actualReturnTime<=j.B,'P3:Unqualified','Only audited on-time success may be inherited.');
    if candidate.fineAuditSelected, assert(candidate.fineAuditPassed); end
    source=candidate; origin=kind; return;
   end
  end
  source=same; origin=sameOrigin;
 end
 function recordView(source,origin)
  inherited=source.B<job.B;
  assert(~inherited||source.finalQualified,'P3:FailureCarry','Failed output cannot propagate.');
  if isfield(r,'expectedNoNewMeasurements')&&r.expectedNoNewMeasurements
   assert(inherited==(job.B>5),'P3:CarryTest','Incorrect success propagation.');
   if inherited, assert(source.B==5&&source.finalQualified); end
  end
  v=struct('evaluationId',job.runId,'targetBudget',job.B,'taskId',job.spec.taskId,'method',job.method, ...
   'repeat',job.repeat,'trainingSeed',job.trainingSeed,'samplingSeed',job.samplingSeed,'scopeHash',r.scopeHash, ...
   'sourceRunId',source.runId,'sourceBudget',source.B,'inherited',inherited,'origin',origin, ...
   'usesMeasurementAtTargetBudget',~inherited,'targetBudgetWasMeasured',isfile(committed)||(strcmp(job.method,'ihsid')&&job.B<20), ...
   'finalQualified',source.finalQualified,'sourceRow',source);
  file=fullfile(r.viewOutput,[job.runId,'.json']); dpWriteJson([file,'.tmp'],v); movefile([file,'.tmp'],file,'f');
 end
 function heartbeat(stage)''')
(H/'p3CarryBatch.m').write_text(s,encoding='utf-8')

s=(H/'summarize.py').read_text(encoding='utf-8')
s=s.replace("p=read(OUT/'protocol.json');assert read(OUT/'formal/audit.json')['passed'];data=OUT/'analysis/data'", "p=read(OUT/'protocol_carry.json');assert read(OUT/'carry/audit.json')['passed'];data=OUT/'analysis_carry/data'")
s=s.replace("rows=[read(OUT/'formal/calls'/j['runId']/'result.json') for j in p['jobs']]", "views=[read(OUT/'carry/evaluations'/(j['runId']+'.json')) for j in p['jobs']];rows=[]\n    for v in views:\n        r=dict(v['sourceRow']);r['B']=v['targetBudget'];r['evaluationId']=v['evaluationId'];r['inherited']=v['inherited'];r['sourceBudget']=v['sourceBudget'];r['sourceRunId']=v['sourceRunId'];rows.append(r)\n    write(data/'evaluation_sources.json',[{k:v for k,v in x.items() if k!='sourceRow'} for x in views])")
s=s.replace("status='measured',qualified=", "status='budget_coverage',measuredAtBudget=sum(not r['inherited'] for r in rs),inherited=sum(r['inherited'] for r in rs),qualified=")
s=s.replace("OUT/'analysis/provenance.json'","OUT/'analysis_carry/provenance.json'")
s=s.replace("callCount=len(rows)","scopeHash=p['scopeHash'],logicalBudgetCells=len(rows),uniqueReferencedCalls=len({r['sourceRunId'] for r in rows}),inheritedCells=sum(r['inherited'] for r in rows),timingMeaning='Source call duration; not a new measurement at inherited budgets; previous failed attempts are not accumulated'")
(H/'report_summarize_carry.py').write_text(s,encoding='utf-8')

s=(H/'report_figures.py').read_text(encoding='utf-8')
s=s.replace("OUT/'analysis", "OUT/'analysis_carry").replace("OUT/'protocol.json'","OUT/'protocol_carry.json'")
s=s.replace('合格轨迹返回率 / %','已验证合格轨迹覆盖率 / %').replace('不同规划时间预算下的轨迹合格率','不同预算上限下的已验证轨迹合格率')
s=s.replace('全部438次调用均计入平均值（包含失败）；各阶段耗时不重复计算，表中单位为秒','438个任务区组；沿用短预算成功轨迹的实测耗时，非新增调用；不累计前序失败，单位为秒')
s=s.replace("ylabel='调用次数'","ylabel='任务区组数'").replace('每方法每预算438次调用。所有结果来自本轮重新运行。','每方法每预算438个任务区组。短预算最终合格后沿用至更大预算；继承结果不计作新调用。IHSID的1、2、5、10秒使用既有记录，仅补测不能沿用成功结果的20秒。')
s=s.replace('五方法无回退重测图册.pdf','五方法预算成功沿用图册.pdf')
(H/'report_figures_carry.py').write_text(s,encoding='utf-8')
print('Carry scheduler and provenance-aware report variants generated.')
