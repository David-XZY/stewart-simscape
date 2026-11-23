"""Replace only timing panels with reproducibly selected, matched actual calls."""
import collections
import gzip
import hashlib
import json
import pathlib
import shutil
import subprocess
import zipfile

import numpy as np
from prepare import ROOT, OUT as CAMPAIGN

SOURCE = CAMPAIGN / 'analysis_carry'
OUT = CAMPAIGN / 'analysis_case_timing_v1'
FIG = OUT / 'figures'
FIG.mkdir(parents=True, exist_ok=True)
M = ['ddpm_basic', 'ddpm_signature', 'regression', 'regression_signature', 'ihsid']
STAGES = {'input_and_endpoint':'输入/编码','outer_input':'输入/编码','encoding_and_retrieval':'输入/编码',
          'generation':'生成','generation_other':'生成','physical':'检查','quality':'检查','acceptance_other':'检查',
          'repair':'精修','repair_exception':'精修','solver':'NLP','solver_initialization':'NLP',
          'solver_build':'NLP','solver_other':'NLP','ihsid_including_internal_acceptance':'NLP'}
KEYS = ['输入/编码','生成','排序','检查','精修','NLP','其他']

def seq(value):
    return value if isinstance(value,list) else [value] if isinstance(value,dict) else []

def write(name, value):
    (OUT/name).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf8')

def stages(row):
    result = dict.fromkeys(KEYS, 0.)
    result['其他'] = row['otherSeconds']
    for event in seq(row['events']):
        result[STAGES.get(event['stage'], '其他')] += event['seconds']
    assert abs(sum(result.values())-row['actualReturnTime']) < 1e-6
    return result

with gzip.open(SOURCE/'data/all_calls.jsonl.gz', 'rt', encoding='utf8') as f:
    rows = [json.loads(line) for line in f]

diagnostics = []
for budget in [1,2,5,10,20]:
    for method in ['regression','regression_signature']:
        failed = [r for r in rows if r['B']==budget and r['method']==method and not r['finalQualified']]
        diagnostics.append(dict(B=budget, method=method, failures=len(failed),
            returnReasons=dict(collections.Counter(r['reason'] for r in failed)),
            repairStopReasons=dict(collections.Counter(seq(r['repairBlocks'])[-1]['reason'] if seq(r['repairBlocks']) else 'none' for r in failed)),
            lastAcceptanceReasons=dict(collections.Counter(seq(r['attempts'])[-1]['reason'] if seq(r['attempts']) else 'none' for r in failed)),
            returnSecondsQuantiles=dict(zip(['min','median','max'],np.quantile([r['actualReturnTime'] for r in failed],[0,.5,1]).tolist())),
            calls=[dict(taskId=r['taskId'],repeat=r['repeat'],sourceRunId=r['sourceRunId'],actualReturnTime=r['actualReturnTime'],
                        attempts=r['attempts'],repairBlocks=r['repairBlocks']) for r in failed]))
write('regression_failure_diagnosis.json',diagnostics)

selected = {}
provenance = []
for budget in [2,10]:
    groups = collections.defaultdict(dict)
    for row in rows:
        if row['B']==budget:
            groups[row['taskId'],row['repeat']][row['method']] = row
    eligible = []
    for key, group in sorted(groups.items()):
        if len(group)!=5 or any(r['inherited'] or r['sourceBudget']!=budget for r in group.values()):
            continue
        if not all(group[m]['actualReturnTime']>=.95*budget for m in ['ddpm_basic','ddpm_signature','ihsid']):
            continue
        assert len({(r['trainingSeed'],r['samplingSeed']) for r in group.values()})==1
        eligible.append((key,group))
    assert eligible
    # Use the modal environment/shape among eligible matched calls, then the
    # observed profile closest to the componentwise median. No outcome tuning.
    stratum = lambda g:(g[M[0]]['job']['spec']['family'],g[M[0]]['payloadShape'])
    counts = collections.Counter(stratum(g) for _,g in eligible)
    modal = sorted(counts, key=lambda k:(-counts[k],k))[0]
    pool = [(k,g) for k,g in eligible if stratum(g)==modal]
    vectors = np.array([[stages(g[m])[s]/budget for m in M for s in KEYS] for _,g in pool])
    distances = np.abs(vectors-np.median(vectors,axis=0)).sum(axis=1)
    idx = int(np.argmin(distances))
    key, group = pool[idx]
    selected[budget] = group
    provenance.append(dict(B=budget,taskId=key[0],repeat=key[1],eligibleMatchedCalls=len(eligible),
        modalStratum=modal,modalStratumCalls=len(pool),selectionDistance=float(distances[idx]),
        selectionRule='All five methods same task/block; no inherited rows; both DDPM and IHSID >=95% budget. Modal family/shape, then minimum L1 distance to median stage profile; lexical tie break.',
        methods=[dict(method=m,runId=group[m]['sourceRunId'],sourceBudget=group[m]['sourceBudget'],
                      actualReturnTime=group[m]['actualReturnTime'],finalQualified=group[m]['finalQualified'],
                      reason=group[m]['reason'],stageSeconds=stages(group[m]),candidateGenerated=group[m]['candidateGenerated']) for m in M]))
    print(json.dumps(provenance[-1],ensure_ascii=True))
write('selected_cases.json',provenance)
write('selected_actual_calls.json',[selected[b][m] for b in [2,10] for m in M])

# Preserve the four other panels byte for byte. Reuse the original plotting
# block to retain fonts, colors, table layout and method ordering.
plot_source = (ROOT/'analysis/payload_pair2_nofallback/report_figures_carry.py').read_text(encoding='utf8')
prefix = plot_source[:plot_source.index("rates=read('success_summary')")]
prefix = prefix.replace("OUT=CAMPAIGN/'analysis_carry/figures'", "OUT=CAMPAIGN/'analysis_case_timing_v1/figures'")
block = plot_source[plot_source.index('# Horizontal stage bars;'):plot_source.index("categories=['直接成功'")]
old = "r=next(r for r in timing if r['method']==m and r['B']==b);assert r['n']==438 and r['stageMeanSeconds']['重新计时']==0\n  matrix.append([r['stageMeanSeconds'][k] for k in stageKeys]);totals.append(r['totalMeanSeconds'])"
new = "r=selected[b][m]; matrix.append([stages(r)[k] for k in stageKeys]);totals.append(r['actualReturnTime'])"
assert old in block
block = block.replace(old,new).replace("xlabel='平均耗时 / s'", "xlabel='实际耗时 / s'")
block = block.replace("各方法的平均耗时构成", "典型困难任务的单次耗时构成")
block = block.replace("fig.text(.56,.925,'全部438次调用均计入平均值（包含失败）；各阶段耗时不重复计算，表中单位为秒',ha='center',fontsize=12,color='#415461')",
    "fig.text(.56,.925,f\"任务 {selected[b][M[0]]['taskId']}，区组 {selected[b][M[0]]['repeat']+1}；各方法均为同一任务的单次实测\",ha='center',fontsize=11,color='#415461')")
block = block.replace("f'{num}_{b}秒平均耗时分解'", "f'{num}_{b}秒典型案例耗时分解'")
env = dict(selected=selected,stages=stages)
exec(compile(prefix+'\n'+block, str(__file__)+'::original_style', 'exec'),env)
for path in (SOURCE/'figures').iterdir():
    if path.name[:2] in ['01','02','05','06'] and path.suffix in ['.png','.svg']:
        shutil.copy2(path,FIG/path.name)
        assert hashlib.sha256(path.read_bytes()).digest()==hashlib.sha256((FIG/path.name).read_bytes()).digest()

(OUT/'index.html').write_text('<meta charset="utf-8"><title>五方法结果图：典型案例耗时</title>'+''.join(
    '<img style="width:100%;max-width:1400px" src="figures/'+p.name+'">' for p in sorted(FIG.glob('*.png'))),encoding='utf8')

marker = 'C:/Users/David_X/.cache/codex-runtimes/codex-primary-runtime/plugins/openai-primary-runtime/plugins/pdf/skills/pdf/container_tools/mark_artifact_operation_started.mjs'
subprocess.run(['node',marker,'--operation-kind','create','--expected-output-count','1','--output-format','pdf'],check=True)
import fitz
pdf = ROOT/'output/pdf/五方法精选结果图册_典型案例耗时.pdf'
doc = fitz.open()
for path in sorted(FIG.glob('*.png')):
    page = doc.new_page(width=1190.55,height=841.89)
    page.insert_image(fitz.Rect(22,22,1168.55,819.89),filename=str(path),keep_proportion=True)
assert len(doc)==6
doc.save(pdf)
doc.close()
qa = OUT/'pdf_qa'
qa.mkdir(exist_ok=True)
subprocess.run(['pdftoppm','-scale-to','1600','-png',str(pdf),str(qa/'page')],check=True)
archive = CAMPAIGN/'五方法图表_典型案例耗时.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    z.write(pdf,pdf.name)
    for p in sorted(OUT.rglob('*')):
        if p.is_file() and 'pdf_qa' not in p.parts:z.write(p,'analysis/'+str(p.relative_to(OUT)))
    z.write(__file__,'code/'+pathlib.Path(__file__).name)
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
print('PDF:',str(pdf),'ZIP bytes:',archive.stat().st_size)
