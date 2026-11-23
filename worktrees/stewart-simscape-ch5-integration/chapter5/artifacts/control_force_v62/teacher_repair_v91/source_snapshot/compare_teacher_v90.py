"""Recompute before/after call counts and report the complete safety evidence."""
from pathlib import Path
import argparse
import csv
import hashlib
import html
import json
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'chapter5/artifacts/control_force_v62'
OUT = BASE / 'teacher_repair_v90'


def cells(value):
    if isinstance(value, dict):
        return [value]
    if value is None:
        return []
    return list(np.asarray(value, dtype=object).reshape(-1))


def planning(path):
    data = loadmat(path, variable_names=['plans', 'records'], simplify_cells=True)
    plans, records = cells(data['plans']), cells(data['records'])
    stats = dict(planning_checks=len(plans), ihsid_calls=0, primary_calls=0, backup_calls=0,
                 start_records=0, solver_calls=0, precheck_only_starts=0, solver_failures=0,
                 postcheck_rejections=0, solver_seconds=0., planning_seconds=0., retreat_reuse_checks=0, calls=[])
    stats['planning_seconds'] = sum(float(z['planning_seconds']) for z in records)
    for entry in plans:
        stats['retreat_reuse_checks'] += entry['reference_decision'] == 'retained_checked_retreat_while_goal_hold_obstructed'
        for name in ['plan', 'backup_plan']:
            p = entry.get(name)
            if not isinstance(p, dict):
                continue
            attempts = cells(p.get('attempts'))
            if not attempts:
                continue
            stats['ihsid_calls'] += 1
            stats['primary_calls' if name == 'plan' else 'backup_calls'] += 1
            call = dict(time=float(entry['time']), purpose=name, accepted=bool(p['accepted']),
                        solver_calls=0, precheck_only_starts=0, seconds=float(p['seconds']))
            for a in attempts:
                stats['start_records'] += 1
                called = bool(a.get('solver_called', True))
                if called:
                    stats['solver_calls'] += 1
                    call['solver_calls'] += 1
                    success = bool(a.get('solver_success', False))
                    stats['solver_failures'] += not success
                    if success and isinstance(a.get('postcheck'), dict) and not a['postcheck']['accepted']:
                        stats['postcheck_rejections'] += 1
                    stats['solver_seconds'] += float(a.get('solver_seconds', 0))
                else:
                    stats['precheck_only_starts'] += 1
                    call['precheck_only_starts'] += 1
            stats['calls'].append(call)
    return stats


def main(version='v90'):
    global OUT
    OUT = BASE / ('teacher_repair_' + version)
    identity = json.loads((OUT / 'identity.json').read_text(encoding='utf-8-sig'))
    for name, expected in {**identity['original_sources'], **identity['sources']}.items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == expected, name
    before = json.loads((BASE / 'teacher_repair_v89/summary.json').read_text(encoding='utf-8-sig'))
    file = OUT / 'summary.json'
    after = json.loads(file.read_text(encoding='utf-8-sig')) if file.exists() else {}
    old = {(r['task'], r['mode']): r for r in before['rows']}
    rows, raw = [], []
    for row in after.get('rows', []):
        key = (row['task'], row['mode']); prior = old[key]
        name = row['task'] + '_' + row['mode'].replace('-', '_') + '.mat'
        cache = OUT / ('v89_counts_' + name.replace('.mat', '.json'))
        if cache.exists():
            a = json.loads(cache.read_text(encoding='utf-8'))
        else:
            a = planning(BASE / 'teacher_repair_v89' / name)
            cache.write_text(json.dumps(a, indent=2), encoding='utf-8')
        equivalent = row.get('status') == 'verified_force_equivalent_replay'
        source_path = Path(row['source_full_run']) if equivalent else OUT / name
        b = planning(source_path)
        if 'planning' in row:
            for metric in ['ihsid_calls', 'solver_calls', 'precheck_only_starts', 'backup_calls']:
                assert row['planning'][metric] == b[metric], metric
        metrics = row.get('metrics', {})
        valid = (row.get('admitted', False) and metrics.get('safe_success', False)
                 and metrics.get('all_constraints_valid', False) and not metrics.get('collision', True)
                 and metrics.get('distance_verified', False) and metrics.get('simulated_end_seconds', 0) >= 15-1e-8)
        actual_wall = None if equivalent else row.get('wall_seconds')
        observed_wall = row['reference_wall_seconds'] if equivalent else actual_wall
        flat = dict(task=key[0], mode=key[1], validation_kind=row.get('validation_kind','new complete V90 Simscape run'),
                    timing_version='v90' if equivalent else version, source_full_run=str(source_path),
                    before_ihsid=a['ihsid_calls'], after_ihsid=b['ihsid_calls'],
                    before_solver=a['solver_calls'], after_solver=b['solver_calls'],
                    before_precheck_starts=a['precheck_only_starts'], after_precheck_starts=b['precheck_only_starts'],
                    before_backup=a['backup_calls'], after_backup=b['backup_calls'],
                    before_wall_seconds=prior['wall_seconds'], after_wall_seconds=actual_wall,
                    observed_reference_wall_seconds=observed_wall,
                    before_planning_seconds=a['planning_seconds'], after_planning_seconds=b['planning_seconds'],
                    retreat_reuse_checks=b['retreat_reuse_checks'], safe_success=bool(valid),
                    before_clearance_mm=prior['metrics']['minimum_clearance']*1000,
                    after_clearance_mm=metrics.get('minimum_clearance', float('nan'))*1000,
                    before_fallback=prior['metrics']['fallback_count'], after_fallback=metrics.get('fallback_count'),
                    before_final_position_mm=prior['metrics']['final_position']*1000,
                    after_final_position_mm=metrics.get('final_position', float('nan'))*1000)
        rows.append(flat)
        raw.append(dict(task=key[0], mode=key[1], before=a, after=b,
                        before_metrics=prior['metrics'], after_metrics=metrics,
                        before_sha256=hashlib.sha256((BASE/'teacher_repair_v89'/name).read_bytes()).hexdigest(),
                        after_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                        validation_kind=flat['validation_kind'],timing_version=flat['timing_version']))
    completed = bool(after.get('completed')) and len(rows) == 3
    report = dict(completed=completed, formal=False, version=version, rows=rows, evidence=raw,
                  all_tested_runs_safe=completed and all(r['safe_success'] for r in rows),
                  timing_scope='Observed serial wall time; V89 is a historical paired task run. Cold bootstrap/model update are reported separately.',
                  safety_scope='Development cases only; dense Simscape/GJK/physical hard-constraint audits, not a general safety proof.',
                  count_definition='IHSID wrapper calls include rejected endpoint prechecks. Solver calls count actual starts, including backup planning; four precheck records are not four NLP solves.')
    (OUT/'comparison.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    if rows:
        with (OUT/'comparison.csv').open('w', newline='', encoding='utf-8-sig') as f:
            writer=csv.DictWriter(f, fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    headings=['任务 / 执行器','IHSID 调用','实际 NLP 求解','15 s 仿真墙钟时间','最小净空 mm','回退','安全成功']
    body=[]
    for r in rows:
        body.append('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in [
            r['task']+' / '+r['mode'],f"{r['before_ihsid']} → {r['after_ihsid']}",
            f"{r['before_solver']} → {r['after_solver']}",
            f"{r['before_wall_seconds']:.2f} → {r['observed_reference_wall_seconds']:.2f} s ({r['timing_version'].upper()})",
            f"{r['before_clearance_mm']:.3f} → {r['after_clearance_mm']:.3f}",
            f"{r['before_fallback']} → {r['after_fallback']}",r['safe_success']])+'</tr>')
    document='''<!doctype html><meta charset="utf-8"><title>VERSION IHSID 重试与安全验证</title>
    <style>body{font:16px system-ui;max-width:1200px;margin:40px auto;line-height:1.7;color:#193047}table{border-collapse:collapse;width:100%}td,th{border:1px solid #cad4de;padding:9px}th{background:#edf3f8}a{color:#12598d}</style>
    <h1>VERSION IHSID 重试与安全验证</h1>'''.replace('VERSION',version.upper())
    document+=f'<p>完整运行状态：{len(rows)}/3；全部完成：{completed}。开发验证，正式实验仍为 0/1440。</p>'
    document+='<table><tr>'+''.join('<th>'+h+'</th>' for h in headings)+'</tr>'+''.join(body)+'</table>'
    document+='<p>V90 仅改变基线重规划调度：目标未变且目标保持预测未通过时，先检查现有退避轨迹的跟踪误差、剩余全过程和至少 1.2 s 保持；全部通过才复用。目标改变、跟踪偏差过大或退避检查失败均退出复用。</p>'
    if version=='v91':
        document+='<p>V90 首组运行在 14.16 s 出现加速度超限，原始失败结果完整保留。V91 额外修复失败安全投影：当前及输入延迟内的不可控状态按原物理加速度上限 1.2 m/s² 检查；未来状态与执行器瞬态仍按原预留上限 1.188 m/s² 检查。原容差、碰撞、力、变化率和其他硬约束不变，未给 CBF 添加松弛。V89 成功命令直接沿用。</p>'
        document+='<p><b>验证方式逐项区分：</b>发生控制变化或原运行失败的条件重新执行完整 V91 Simscape；通过全部 751 拍力指令等价检查的条件保留 V90 完整时序，其计时仍明确归属 V90。等价回放不是新增完整仿真，不将这些计时合并为同一版本的正式结果。</p>'
        document+='<p><a href="equivalence.json">751 拍控制输出等价检查</a> · <a href="causal_reserve_replay.json">失效状态与反例检查</a></p>'
    else:
        document+='<p>物理限值、检查容差、四组初值、QP 和 LQI 未修改。</p>'
    document+='<p>调用数包含解析预检查；实际 NLP 求解数另列，包含退避规划。旧版仅统计主规划四初值的 planning_attempts 不能替代实际调用数。</p>'
    document+='<p>安全结论限定为固定三组 15 s 开发任务，逐组保留全部失败、回退、净空和硬约束结果。离线同步仿真的运动时间不代表墙钟实时能力。V89 时间来自已保存的相同任务运行，非本轮交错计时。</p>'
    document+='<p><a href="comparison.json">完整指标和逐次调用记录</a> · <a href="comparison.csv">统计 CSV</a> · <a href="identity.json">源码身份</a> · <a href="../teacher_failure_v83/retreat_replay_v90.json">保存状态回放</a></p>'
    (OUT/'index.html').write_text(document, encoding='utf-8')
    print(json.dumps(dict(completed=completed, rows=rows), ensure_ascii=False, indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--version',choices=['v90','v91'],default='v90')
    main(parser.parse_args().version)
