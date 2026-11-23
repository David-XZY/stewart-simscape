"""Verify actual training loader, immutable parent rows, coverage and audit identity."""
import collections
import hashlib
import html
import json
import shutil
from pathlib import Path
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[2]
NAME='payload_condition_expansion_v2_longtime_20260926'
sys.path.insert(0,str(ROOT/'chapter3/payload_generalization'))

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):Path(p).write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')

def main():
    from pg_learning import validate_dataset
    out=ROOT/'results/chapter3'/NAME
    parent_path=ROOT/'results/chapter3/payload_formal_dataset_v1/dataset.json'
    old=read(parent_path); data=read(out/'dataset.json'); summary=read(out/'dataset_report.json')
    assert summary['status']=='completed'
    rows=validate_dataset(data)
    assert sha(parent_path)==read(out/'identity.json')['parentSha256']
    assert sha(ROOT/'analysis/payload_expansion/pgExpandDataset.m')==read(out/'identity.json')['implementationSha256']
    assert data['validationTasks']==old['validationTasks']
    assert rows[:len(old['rows'])]==old['rows']
    assert len({r['taskId'] for r in rows})==len(rows)
    assert len({json.dumps(r['context'],separators=(',',':')) for r in rows})==len(rows)
    audits=read(out/'independent_audits.json')
    assert len(audits)==len(rows) and all(a['passed'] for a in audits)
    assert {a['taskId'] for a in audits}=={r['taskId'] for r in rows}
    lib=read(out/'library.json')['records']
    assert len(lib)==len(rows) and all((out/r['file']).is_file() for r in lib)
    selected=read(out/'selected_tasks.json')
    assert len(selected['tasks'])==300
    strata=collections.Counter((t['family'],t['taskClass'],t['shape']) for t in selected['tasks'])
    assert len(strata)==12 and set(strata.values())=={25}
    durations=collections.Counter(t['parameters'][3] for t in selected['tasks'])
    assert durations=={5:60,7.5:60,10:60,12.5:60,15:60}
    assert len({t['baseConditionId'] for t in selected['tasks']})==60
    checks=[read(p) for p in sorted((out/'checks').glob('*.json'))]
    new=[r for r in checks if r['origin']=='new_condition']
    recovered=[r for r in checks if r['origin']=='historical_recheck' and r['accepted']]
    assert len(new)==300
    assert sum(r['accepted'] for r in new)==summary['newConditionsAccepted']
    assert len(recovered)==summary['historicalConditionsRecovered']
    assert all(r['task']['split']=='train' for r in rows[len(old['rows']):])
    for row in rows[len(old['rows']):]:
        cell=row['task']['payloadRegionId']
        assert int(hashlib.sha256(cell.encode()).hexdigest()[:8],16)%10<7
    recovery_reasons=collections.Counter(r['historicalReason'] for r in recovered)
    failed=collections.Counter(r['reason'] for r in new if not r['accepted'])
    group=[]
    for key in sorted(strata):
        items=[r for r in new if (r.get('family'),r.get('taskClass'),r.get('shape'))==key]
        group.append(dict(family=key[0],taskClass=key[1],shape=key[2],attempted=len(items),accepted=sum(r['accepted'] for r in items)))
    physical_only=[r for r in checks if r['accepted'] and not r['historicalPassed']]
    physical_report=read(out/'physical_acceptance_report.json')
    physical_lib=read(out/'physical_trajectory_library.json')['records']
    assert len({r['taskId'] for r in physical_lib})==len(physical_lib)
    assert len(physical_lib)==len(rows)+physical_report['additionalRawSafeConditions']
    assert all(r['physicalPassed'] for r in physical_report['rawOnly'])
    assert all((out/r['file']).is_file() for r in physical_lib)
    physical_ids={r['taskId'] for r in physical_lib}
    spline_ids={r['taskId'] for r in rows}
    physical_failed=collections.Counter(r.get('rawPhysical',{}).get('reason',r['reason']) for r in new if r['taskId'] not in physical_ids)
    duration_results=[]
    for duration in [5,7.5,10,12.5,15]:
        subset=[t for t in selected['tasks'] if t['parameters'][3]==duration]
        duration_results.append(dict(durationSeconds=duration,conditions=len(subset),
            physicallyAccepted=sum(t['taskId'] in physical_ids for t in subset),
            trainingSplineAccepted=sum(t['taskId'] in spline_ids for t in subset)))
    findings=dict(summary=summary,recoveryHistoricalReasons=dict(recovery_reasons),newFailureReasons=dict(failed),
        physicalAcceptance=physical_report,
        newConditionsByStratum=group,physicallyAcceptedDespiteHistoricalGate=len(physical_only),
        historicalAttemptsRechecked=sum(r['origin']=='historical_recheck' for r in checks),
        independentAuditLabels=len(audits),parentRowsExact=True,validationTasksExact=True,
        actualTrainingLoaderPassed=True,uniqueTaskIds=True,uniqueNumericContexts=True,
        freshTrainMacrocellCheckPassed=True,datasetSha256=sha(out/'dataset.json'))
    findings['newConditionDurations']=dict(durations)
    findings['durationResults']=duration_results
    findings['newSplineFailureReasons']=dict(failed)
    findings['newPhysicalFailureReasons']=dict(physical_failed)
    findings['newPhysicallyAcceptedConditions']=sum(r['physicallyAccepted'] for r in duration_results)
    findings['historicalPhysicallyAcceptedConditions']=physical_report['physicallyAcceptedTrainConditions']-findings['newPhysicallyAcceptedConditions']
    findings['timeScalingRecovery']=read(out/'time_scaling_report.json')
    summary['secondsScope']='Final solver/revalidation invocation only; excludes earlier scheduling sessions and postprocessing.'
    write(out/'dataset_report.json',summary)
    write(out/'progress.json',dict(summary,phase='all_physical_and_encoding_audits_completed',
        physicallyAcceptedTrainConditions=physical_report['physicallyAcceptedTrainConditions']))
    write(out/'verification.json',findings)
    table=''.join('<tr>'+''.join(f'<td>{html.escape(str(x))}</td>' for x in [g['family'],g['taskClass'],g['shape'],g['attempted'],g['accepted']])+'</tr>' for g in group)
    reasons=html.escape(json.dumps(dict(recovery_reasons),ensure_ascii=False,indent=2))
    duration_table=''.join('<tr>'+''.join(f'<td>{v}</td>' for v in row.values())+'</tr>' for row in duration_results)
    doc=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>独立工况扩充与物理验收</title>
<style>body{{font:17px/1.65 system-ui,"Microsoft YaHei",sans-serif;max-width:1050px;margin:45px auto;color:#173042;padding:0 25px}}h1{{font-size:30px}}table{{border-collapse:collapse;width:100%}}th,td{{padding:9px 14px;border-bottom:1px solid #d5e1e7;text-align:left}}.big{{padding:20px;background:#edf7f4;font-size:23px}}code,pre{{white-space:pre-wrap;font-size:14px}}small{{color:#536b7b}}</style>
<h1>独立工况扩充与物理验收</h1><p class="big">训练集：175 → {summary['trainConditions']} 个工况；物理合格轨迹库：{physical_report['physicallyAcceptedTrainConditions']} 个训练工况。每个工况恰好 1 条轨迹。</p>
<p>历史失败工况找回 {summary['historicalConditionsRecovered']} 个；新增 300 个端点合法工况，{summary['newConditionsAccepted']} 个取得合格样条轨迹。验证标签仍为 53 条，验证评估条件仍为 120 个。没有重新训练模型。</p>
<p>60 组起终点/场景/载荷组合各覆盖 5、7.5、10、12.5、15 秒，每档 60 个工况，每个环境×运动×载荷形状分组中各档均为 5 个。同组按长时长到短时长求解并复用合格曲线作为初值，各时长独立验收。历史已有轨迹保留其原定时长。</p>
<p>其中 {summary.get('timeScaledRecovered',0)} 个新增标签来自同组已合格 IHSID 曲线的时长重参数化，并在目标时长下重新通过完整物理验收；没有将它们标记为目标工况下 IHSID 收敛成功。</p>
<h2>验收口径</h2><p>取消求解器成功状态、优化残差、120 秒内完成和拟合模式距离等对最终曲线入库的额外限制。不设置最优性或代价门槛。一次 IHSID 求解仍设 120 秒资源上限，返回曲线的独立验收不计入该上限。</p>
<p>沿用原 pgValidate：杆长、杆速、杆加速度、执行器力、奇异性、起终点位置/姿态和静止条件、总时长、载荷与声明机构包络的 10 mm 安全间距。检查间隔 5 ms，近障碍加密至 1 ms；保留既有数值容差。这是模型上的稠密检查，不是连续时间证明或实机跟踪验证。</p>
<p>存储的最终教师轨迹就是通过验收的 16 控制点样条。历史 IHSID 返回曲线是否合格单独记录，不能由样条通过反推其原始离散解合格。</p>
<p>另有 {physical_report['additionalRawSafeConditions']} 个工况的原始轨迹通过物理验收，但尚未得到合格的 16 控制点编码。这些轨迹已保存到物理轨迹库，算作验收通过，不计入当前固定输出维度的训练标签。</p>
<h2>各时长结果</h2><table><tr><th>时长/秒</th><th>工况数</th><th>物理验收通过</th><th>可直接训练的样条</th></tr>{duration_table}</table>
<h2>新增工况分布</h2><table><tr><th>环境</th><th>运动</th><th>形状</th><th>求解工况</th><th>合格标签</th></tr>{table}</table>
<h2>历史恢复来源</h2><pre>{reasons}</pre><p>新增工况的样条编码未通过原因（包括原始轨迹已合格者）：{html.escape(str(dict(failed)))}</p>
<p>新增工况中，尚未取得物理合格轨迹的原因：{html.escape(str(dict(physical_failed)))}。物理合格 {findings['newPhysicallyAcceptedConditions']}/300 个；历史物理合格 {findings['historicalPhysicallyAcceptedConditions']} 个。</p>
<h2>验证</h2><p>{len(audits)} 条最终标签已独立物理复核。原 228 条标签及全部验证条件逐字段保持一致；无重复工况 ID 或完全相同的条件向量；新训练载荷参数仍满足原训练宏单元划分；通过实际训练程序的数据校验。</p>
<p><a href="dataset.json">训练数据</a> · <a href="verification.json">完整核验结果</a> · <a href="selected_tasks.json">300 个新工况</a> · <a href="independent_audits.json">逐条物理验收</a> · <a href="physical_trajectory_library.json">全部物理合格轨迹库</a></p>
<small>数据 SHA-256：{sha(out/'dataset.json')}。新数据独立保存，历史模型和测试结果保持原样；本结果不表示扩充后模型性能已提高。</small></html>'''
    (out/'report.html').write_text(doc,encoding='utf-8')
    implementation=out/'implementation';implementation.mkdir(exist_ok=True)
    for p in (ROOT/'analysis/payload_expansion').iterdir():
        if p.suffix in ['.py','.m'] or p.name=='candidates.json':shutil.copy2(p,implementation/p.name)
    files=[p for p in out.iterdir() if p.suffix in ['.json','.mat','.html'] and p.name!='delivery_manifest.json']
    files+=list((out/'checks').glob('*.json'))
    files+=list((out/'checks_before_time_scaling').glob('*.json'))
    files+=list(implementation.iterdir())
    hashes={str(p.relative_to(out)).replace('\\','/'):sha(p) for p in files}
    write(out/'delivery_manifest.json',dict(files=hashes))
    archive=out.parent/(NAME+'_delivery.zip')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for p in files+[out/'delivery_manifest.json']:z.write(p,p.relative_to(out))
    with zipfile.ZipFile(archive) as z:assert z.testzip() is None
    print(json.dumps(dict(**findings,archive=str(archive),archiveSha256=sha(archive)),ensure_ascii=False))

if __name__=='__main__':main()
