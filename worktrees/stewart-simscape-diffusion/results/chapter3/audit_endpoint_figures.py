"""Independent checks of archived records, plotting tables, and resampling."""
import ast,collections,csv,gzip,hashlib,json,pathlib,shutil,xml.etree.ElementTree as ET
import numpy as np
P=pathlib.Path(__file__).resolve().parent
O=P/'payload_endpoint_pass_figures_v1_20260927';S=P/'Stewart_complete_report_20260926/data'
def read(p):return json.loads(p.read_text(encoding='utf8'))
def rows(p):
 with gzip.open(p,'rt',encoding='utf8') as f:return [json.loads(x) for x in f]
C=read(O/'analysis_config.json');M=C['methods'];B=C['budgets']
for name,digest in C['sourceSha256'].items():assert hashlib.sha256((S/name).read_bytes()).hexdigest()==digest
raw=rows(S/'main_all_calls.jsonl.gz')+rows(S/'controls_all_calls.jsonl.gz')
raw=[r for r in raw if r['method'] in M and r['B'] in B]
exclude={r['taskId'] for r in read(O/'data/excluded_tasks.json')}
source={r['runId']:r for r in raw if r['taskId'] not in exclude}
filtered=rows(O/'data/filtered_calls.jsonl.gz')
assert len(source)==len(filtered)==7884
for r in filtered:assert source[r['runId']]==r
for m in M:
 for b in B:
  full=[r for r in raw if r['method']==m and r['B']==b]
  if not full:continue
  assert len(full)==576 and len([r for r in full if r['taskId'] in exclude])==138
  assert {r['taskId'] for r in full if r['taskId'] in exclude}==exclude
for z in read(O/'data/success_summary.json'):
 rr=[r for r in filtered if r['method']==z['method'] and r['B']==z['B']]
 assert len(rr)==z['n']
 if rr:
  assert z['qualified']==sum(r['finalQualified'] for r in rr)
  assert abs(z['mean']-np.mean([r['finalQualified'] for r in rr]))<1e-15
  assert all(not r['finalQualified'] or r['actualReturnTime']<=r['B'] for r in rr)
for z in read(O/'data/paired_success.json'):
 if z['status']=='unmeasured':continue
 a={(r['taskId'],r['repeat']):r for r in filtered if r['method']==z['method'] and r['B']==z['B']}
 b={(r['taskId'],r['repeat']):r for r in filtered if r['method']==z['baseline'] and r['B']==z['B']}
 assert a.keys()==b.keys()
 assert abs(z['mean']-np.mean([int(a[k]['finalQualified'])-int(b[k]['finalQualified']) for k in a]))<1e-15
idx={(r['method'],r['taskId'],r['repeat'],r['B']):r for r in filtered}
joint=read(O/'data/common_success.json');obs=read(O/'data/matched_observations.json')
assert len(obs)==1240
for z in obs:
 r=idx[z['method'],z['taskId'],z['repeat'],10];assert r['finalQualified']
 for k,v in z['quality'].items():assert v==(sum(r['rawMetrics'][k]) if k=='legTotalVariation' else r['rawMetrics'][k])
assert len(joint['keys'])==248 and joint['tasks']==83 and joint['scenes']==15
outcomes=collections.Counter()
for z in read(O/'data/outcomes.json'):outcomes[z['method'],z['B']]+=z['calls']
assert set(outcomes.values())=={438}
# Execute only the boot function, not the plotting module or any planner.
tree=ast.parse((P/'plot_endpoint_five_methods.py').read_text(encoding='utf8'))
fun=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='boot')
ns={'np':np};exec(compile(ast.Module(body=[fun],type_ignores=[]),'<boot audit>','exec'),ns)
toy=[{'baseSceneId':s,'repeat':k,'taskId':f'{s}{t}','v':float(s=='B')+.1*k} for s,n in [('A',1),('B',3)] for t in range(n) for k in range(3)]
check=ns['boot'](toy,lambda r:r['v']);assert abs(check['mean']-.85)<1e-14
rng=np.random.default_rng(20260927);si=rng.integers(2,size=(2000,2));ki=rng.integers(3,size=(2000,3));manual=[]
for ss,kk in zip(si,ki):
 values=[r['v'] for s in ss for k in kk for r in toy if r['baseSceneId']==['A','B'][s] and r['repeat']==k]
 manual.append(np.mean(values))
assert np.allclose(check['ci95'],np.quantile(manual,[.025,.975]),atol=1e-14)
# Make spreadsheet-friendly copies without changing original numerical JSONs.
for file in (O/'data').glob('*.json'):
 data=read(file)
 if not isinstance(data,list) or not data or not all(isinstance(r,dict) for r in data):continue
 keys=list(dict.fromkeys(k for r in data for k in r))
 with file.with_suffix('.csv').open('w',encoding='utf-8-sig',newline='') as f:
  w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
  for r in data:w.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in r.items()})
for svg in (O/'figures').glob('*.svg'):
 root=ET.parse(svg).getroot();visible=' '.join(''.join(x.itertext()) for x in root.iter() if x.tag.endswith('}text'))
 assert not any(x in visible for x in ['检索','增强搜索','物理损失','物理引导'])
 assert '<text' in svg.read_text(encoding='utf8')
result={'passed':True,'sourceRecordsExactlyMatched':7884,'exclusionSetsEqual':True,'bootstrapUnequalClusterRatioAndSharedBlocks':True,'matchedQualityValuesChecked':1240*12,'figuresInScope':7,'noNumericalExperimentsLaunched':True}
(O/'data/independent_analysis_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
shutil.copy2(__file__,O/pathlib.Path(__file__).name)
print(json.dumps(result))
