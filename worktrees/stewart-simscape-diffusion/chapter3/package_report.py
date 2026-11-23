"""Package the completed report and its evidence, capped at 50,000,000 bytes."""
import argparse
from collections import Counter
import csv
import hashlib
import html
import json
from pathlib import Path
import statistics
import zipfile


CORE = {'id', 'goal_position', 'goal_attitude', 'layout', 'joint'}
LABELS = {
    'geo_ihsid': '几何初值＋IHSID', 'retrieve_ihsid': '近邻初值＋IHSID',
    'diff_ihsid': '扩散初值＋IHSID', 'geo_fast': '几何候选＋检查及精修',
    'retrieve_fast': '近邻轨迹＋检查及精修', 'diff_fast': '扩散候选＋检查及精修',
    'previous_ihsid': '上一条成功轨迹热启动 IHSID',
}


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    report = run / 'report'
    summary = json.loads((report / 'summary.json').read_text(encoding='utf-8-sig'))
    assert summary['complete'] and not summary['integrityErrors']
    rows = read_csv(report / 'all_jobs.csv')
    assert len(rows) == summary['scheduled'] == summary['completed']
    assert sum(r['passed'] == 'True' for r in rows) == summary['qualified']
    assert dict(Counter(r['status'] for r in rows)) == summary['statuses']

    def selected(method, categories):
        return [r for r in rows if r['method'] == method and r['category'] in categories and r['purpose'] == 'main']

    def table(methods, categories):
        result = []
        for method in methods:
            all_rows = selected(method, categories)
            good = [r for r in all_rows if r['passed'] == 'True']
            med = lambda key: statistics.median(float(r[key]) for r in good)
            result.append([LABELS[method], f'{len(good)}/{len(all_rows)}',
                           f'{med("generationSeconds"):.3f}', f'{med("onlineSeconds"):.3f}',
                           f'{med("score.primaryScore"):.2f}'])
        return result

    tables = [
        ('40 个独立任务，每种方法实际调用 120 次', table(list(LABELS)[:6], CORE)),
        ('连续变化任务：8 条序列，每种方法实际调用 264 次',
         table(['retrieve_fast', 'diff_fast', 'previous_ihsid'], {'sequence'})),
    ]
    headings = ['方法', '达标/调用', '达标生成中位数/秒', '达标总历时中位数/秒', '达标分数中位数']
    main_diffusion = [r for r in rows if r['method'] == 'diff_fast' and r['purpose'] == 'main']
    stages = Counter(r['acceptedStage'] for r in main_diffusion if r['passed'] == 'True')
    stage_rows = []
    for name, categories in [('独立任务', CORE), ('相关参数扫描', {'duration', 'physical'}), ('连续任务', {'sequence'})]:
        part = selected('diff_fast', categories)
        counts = Counter(r['acceptedStage'] for r in part if r['passed'] == 'True')
        stage_rows.append([name, str(len(part)), str(counts['direct']), str(counts['repair']),
                           str(counts['ihsid']), str(len(part)-sum(counts.values()))])
    pairs = read_csv(report / 'paired_comparisons.csv')
    paired = [r for r in pairs if r['category'] in CORE and r['baseline'] == 'retrieve_fast'
              and r['diffusionPassed'] == 'True' and r['baselinePassed'] == 'True']
    faster = sum(float(r['netSecondsSaved']) > 0 for r in paired)
    higher = sum(float(r['scoreDifference']) > 0 for r in paired)
    scan_only = [r for r in pairs if r['category'] in {'duration', 'physical'} and r['baseline'] == 'retrieve_fast'
                 and r['diffusionPassed'] == 'True' and r['baselinePassed'] == 'False']

    paragraphs = [
        '本批结果支持“扩散能够快速返回达到预定质量的轨迹”，但不支持“扩散全面优于近邻检索，因而不可替代”。'
        '近邻检索在独立任务中达标次数更多，成功生成耗时中位数更低；在连续任务中，近邻和扩散均全部达标，近邻仍更快。',
        f'在独立任务中，扩散与近邻双方成功的 {len(paired)} 对同任务、同重复编号调用里，'
        f'扩散生成耗时更短的有 {faster} 对，主评分更高的有 {higher} 对。这里是调用次数，不是独立场景数。',
        '扩散相对本次完整 IHSID 数值优化流程具有明显的速度优势，但质量分数并非全面更高。'
        '历史轨迹热启动 IHSID 在连续任务中全部达标；其成功生成耗时中位数仍明显大于扩散和近邻。'
        '本次任务变化后重建 NLP 图，因此结论限定于该实现，不代表所有经过参数化缓存优化的 IHSID 实现。',
        f'在相关参数扫描中，扩散较近邻多获得 {len(scan_only)} 次成功调用，来自 '
        f'{len({r["caseId"] for r in scan_only})} 个 3 秒时长条件（scan_a1_T3、scan_a2_T3）。'
        '这是少数基础场景上的局部证据，不能推广为全部短时任务上的优势。',
        f'扩散主方法共 {len(main_diffusion)} 次调用，{sum(stages.values())} 次达标返回：'
        f'{stages["direct"]} 次直接达标、{stages["repair"]} 次轻量修正后达标、'
        f'{stages["ihsid"]} 次 IHSID 回退后达标。下表只统计最终返回阶段；尝试精修但最终失败的调用另见 return_stages.csv。',
    ]
    caveats = [
        '全部 5070 次计划调用已结束，4658 次达标、185 次无合格输出、216 次边界拒绝、11 次硬截止超时；含主实验与消融，不能将这些数合并成某一方法的成功率。汇总完整性检查无错误。',
        '40 个独立任务中有 8 个边界不合法，保留在分母中。每种主方法的 120 次调用中有 96 次边界合法。参数扫描、连续条件和三次重复均不增加独立场景数量。',
        '表中耗时与分数只统计最终达标返回；成功集合不同，应结合达标率、配对表及时间内达标曲线理解。生成耗时扣除独立评价，总历时保留全部在线开销。',
        '主权重为平稳侧重（20%、25%、35%、20%），在验证集按有利于体现扩散优势的规则选择并冻结。完整报告同时展示均衡与路径/执行负担侧重权重，不能只凭主权重声称全面领先。',
        '两种方法统一按 10 mm 物理间隙验收；IHSID 配点内沿用最多 1 mm 的插值保护余量。载荷、惯量和执行器限值未作为当前网络输入，相关实验只能支持检查与后处理适配。',
        '数值稠密检查不等于连续时间安全证明；机械功不是电能消耗。IHSID 是高质量数值参考解，不宣称全局最优。',
    ]
    figures = [
        ('success_and_time.png', '独立任务通过率与耗时（耗时图包含已测得的失败调用）'),
        ('time_to_qualified.png', '在给定时间内返回合格轨迹的比例'),
        ('quality_groups.png', '四组质量负担，数值越低越好'),
        ('history_warm_start.png', '连续任务中成功返回的生成耗时'),
    ]
    md = ['# 第三章实验结论摘要', '', *[p+'\n' for p in paragraphs]]
    body = ['<h1>第三章实验结论摘要</h1>', '<p class="version">chapter3_v3 · 正式实验完成 · 2026-09-19</p>']
    body += [f'<p>{html.escape(p)}</p>' for p in paragraphs]
    all_tables = tables + [('扩散返回阶段（实际调用数）', stage_rows)]
    for title, data in all_tables:
        headers = headings if data is not stage_rows else ['实验层', '调用', '直接达标', '轻量修正达标', 'IHSID 回退达标', '未达标']
        md += ['## '+title, '', '|'+ '|'.join(headers)+'|', '|'+'|'.join(['---']*len(headers))+'|']
        md += ['|'+'|'.join(row)+'|' for row in data]
        md.append('')
        body += ['<h2>'+html.escape(title)+'</h2>', '<div class="table"><table><thead><tr>'+''.join('<th>'+html.escape(h)+'</th>' for h in headers)+'</tr></thead><tbody>']
        body += ['<tr>'+''.join('<td>'+html.escape(cell)+'</td>' for cell in row)+'</tr>' for row in data]
        body.append('</tbody></table></div>')
    md += ['## 解释边界', '', *['- '+p for p in caveats], '', '完整方法、图表与数据见 report/；冻结评分配置见 config/。']
    body += ['<h2>解释边界</h2><ul>', *['<li>'+html.escape(p)+'</li>' for p in caveats], '</ul>']
    body += ['<h2>关键图表</h2>']
    for filename, caption in figures:
        assert (report/filename).exists()
        body.append(f'<figure><img src="report/{filename}" alt="{html.escape(caption)}"><figcaption>{html.escape(caption)}</figcaption></figure>')
    body.append('<p>全部 28 组图提供 PNG 和 SVG；完整方法与论证见 <a href="report/chapter3_materials.md">chapter3_materials.md</a>。逐调用、配对比较、计时和失败数据保留为 CSV。</p>')
    document = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>第三章实验结论摘要</title><style>body{max-width:1100px;margin:40px auto;padding:0 24px;font:16px/1.8 "Microsoft YaHei",sans-serif;color:#172b3a}h1,h2{line-height:1.35}h2{margin-top:36px}.version{color:#536879}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border:1px solid #ccd5dc;padding:9px;text-align:left}th{background:#edf2f6}.table{overflow:auto}figure{margin:28px 0}img{width:100%;height:auto}figcaption{color:#536879}li{margin:10px 0}</style><body>' + ''.join(body) + '</body></html>'

    entries = {'先看这里_结论.html': document.encode('utf-8'), '结论摘要.md': '\n'.join(md).encode('utf-8')}
    for path in sorted(report.iterdir()):
        if path.is_file() and path.suffix in {'.md', '.csv', '.json', '.png', '.svg'}:
            entries['report/'+path.name] = path.read_bytes()
    for filename in ['standard.json', 'core_tests.json', 'cache_tests.json']:
        entries['config/'+filename] = (run/filename).read_bytes()
    manifest = [{'path': name, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()} for name, data in entries.items()]
    entries['文件校验清单.json'] = json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf-8')
    name = run.name+'_报告图表结论'
    destination = run.parent/(name+'.zip')
    assert not destination.exists(), 'Refusing to replace an existing delivery package'
    with zipfile.ZipFile(destination, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as package:
        for filename, data in entries.items():
            package.writestr(name+'/'+filename, data)
    assert destination.stat().st_size < 50_000_000, 'Package exceeds the conservative 50 MB cap'
    with zipfile.ZipFile(destination) as package:
        assert package.testzip() is None
        for item in manifest:
            actual = package.read(name+'/'+item['path'])
            assert hashlib.sha256(actual).hexdigest() == item['sha256']
    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    destination.with_suffix('.zip.sha256').write_text(digest+'  '+destination.name+'\n', encoding='utf-8')
    print(json.dumps({'path': str(destination), 'bytes': destination.stat().st_size,
                      'decimalMB': destination.stat().st_size/1_000_000, 'files': len(entries),
                      'png': sum(k.endswith('.png') for k in entries), 'svg': sum(k.endswith('.svg') for k in entries),
                      'sha256': digest, 'verified': True}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
