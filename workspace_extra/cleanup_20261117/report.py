"""Write the cleanup report and index from the operation and verification records."""
from collections import Counter, defaultdict
import csv
import shutil

from cleanup import ARCHIVE, RECORD, ROOT, load_manifest, read_actions, read_json


def gib(byte_count):
    return f'{byte_count/1024**3:,.3f}'


def main():
    planned = load_manifest()
    inventory = read_json(RECORD/'inventory_notes.json')
    checks = read_json(RECORD/'protection_verification.json')
    actions = list(read_actions())
    actual = {a['path']: a for a in actions if a['event'] in ('removed','skip','missing')}
    actual.update((a['path'], a) for a in actions if a['event']=='removed')
    receipts = [read_json(p) for p in sorted((RECORD/'verified').glob('*.7z.json'))]
    by_archive = defaultdict(list)
    for row in planned:
        if row['action']=='archive':
            by_archive[row['archive']].append(row)
    counts = Counter()
    sizes = Counter()
    remaining = []
    with (RECORD/'清理清单.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        output = csv.writer(stream)
        output.writerow(['原相对路径','原字节数','SHA-256','处置','结果','理由','恢复来源','ZIP 成员路径'])
        for row in planned:
            record = actual.get(row['path'],{})
            status = record.get('event','pending')
            if status=='removed' and (ROOT/row['path']).exists():
                status='reappeared'
            counts[status,row['action']] += 1
            sizes[status,row['action']] += row['bytes']
            if status!='removed':
                remaining.append((row['path'],status,record.get('reason','')))
            backing = row.get('retained',{})
            output.writerow([row['path'],row['bytes'],row['sha256'],row['action'],status,
                             record.get('reason',row['reason']),
                             row.get('archive') or backing.get('path',''),backing.get('member','')])
        for item in inventory['skipped']:
            output.writerow([item['path'],'','','keep','skipped',item['reason'],'',''])
    archive_bytes = sum(r['archive_bytes'] for r in receipts)
    removed_bytes = sum(sizes['removed',key] for key in ('archive','cache','duplicate'))
    overhead = sum(p.stat().st_size for p in RECORD.rglob('*') if p.is_file())
    after = shutil.disk_usage(ROOT).free
    smoke_path = RECORD/'matlab_smoke.log'
    smoke = smoke_path.is_file() and 'CLEANUP MATLAB SMOKE PASSED:' in smoke_path.read_text(encoding='utf-8',errors='replace')
    recovery_path = RECORD/'restore_smoke.txt'
    recovery = recovery_path.is_file() and 'PASS' in recovery_path.read_text()
    evidence_path = RECORD/'selected_evidence.json'
    evidence = read_json(evidence_path) if evidence_path.is_file() else {}
    index = ['# Stewart 历史工程与实验归档','',
        '每个压缩包保留以 Matlab_code 为根的原相对路径。文件可按原目录或单文件恢复。','',
        '| 压缩包 | 原目录 | 文件数 | 原始 GiB | 压缩 GiB | SHA-256 |',
        '|---|---|---:|---:|---:|---|']
    for r in receipts:
        name = r['name']
        index.append(f"| [{name}]({name}) | `{by_archive[name][0]['group']}` | {r['files']:,} | {gib(r['bytes'])} | {gib(r['archive_bytes'])} | `{r['sha256']}` |")
    index += ['', '逐文件来源及恢复方法见 [清理报告](../../cleanup_20261117/清理报告.md)。','']
    (ARCHIVE/'README.md').write_text('\n'.join(index),encoding='utf-8')
    lines = ['# Matlab_code 清理报告','',
        f'从原路径移除 **{gib(removed_bytes)} GiB**，新增压缩归档 **{gib(archive_bytes)} GiB**。'
        f'计入清理记录后，工作区文件内容净减少约 **{gib(removed_bytes-archive_bytes-overhead)} GiB**。','',
        f'D 盘在清理窗口的可用空间从 {gib(inventory["drive_before_bytes"])} GiB 变为 {gib(after)} GiB。同期活动和文件系统分配方式会影响这项读数。','',
        '| 处置 | 完成文件数 | 原路径移除 GiB |','|---|---:|---:|']
    for key,label in [('archive','验证后归档'),('cache','可再生缓存'),('duplicate','已有相同 ZIP 成员')]:
        lines.append(f'| {label} | {counts["removed",key]:,} | {gib(sizes["removed",key])} |')
    lines += ['', 'Git 主仓库和关联工作树继续使用原路径。MPC v8 保留 9 月清单保护的文件，其余候选按索引归档。'
        '旧在线控制实验的文献保留原位。历史轨迹、逐拍矩阵、失败记录和冻结源码留在原路径或完整归档中。','',
        f'共 {len(receipts)} 个压缩包通过完整性检查及逐文件解压 SHA-256 核对。'
        f'保护清单复核 {checks["protected_paths"]:,} 个路径，本次变化 {len(checks["changed_protected_paths"])} 个。'
        f'盘点时已有 {checks["preexisting_protection_differences"]} 个路径与至少一份旧快照有差异。','',
        f'Git 工作树及跟踪文件状态复核{"通过" if checks["git_worktrees_and_tracked_status_unchanged"] else "有变化"}。'
        f'MATLAB 轻量检查{"通过" if smoke else "未通过或尚未完成"}。恢复工具验证{"通过" if recovery else "未通过或尚未完成"}。'
        'MATLAB 检查覆盖入口解析、任务集、SLX 加载、检查点、CasADi、HPIPM、Python 和几何自检。','',
        f'最新 NMPC 结果索引的 {evidence.get("checkedRuns",0)} 项实验经过现有证据检查。'
        '[选定实验核对结果](selected_evidence.json)。','',
        f'清单中仍有 {len(remaining):,} 个候选未处置或原路径重新出现。'
        f'扫描另跳过 {len(inventory["skipped"]):,} 个条目，包括目录联接。','',
        '[归档索引](../_archive/20261117/README.md) · [逐文件清单](清理清单.csv) · [保护复核](protection_verification.json) · '
        '[保护基线](protected_before.json) · [MATLAB 日志](matlab_smoke.log) · [操作日志](actions.jsonl)','',
        '从 Matlab_code 根目录预览恢复列表。','',
        '```powershell',"python .\\cleanup_20261117\\restore.py --prefix 'stewart-simscape-tracking-mpc-v3'",'```','',
        '在同一命令末尾加 `--apply` 还原原路径。恢复工具校验压缩包和文件内容，遇到原路径有不同文件会停止。'
        '缓存由下次运行重新生成。','']
    if remaining:
        lines += ['| 未处置路径（最多列举 15 项） | 状态 | 理由 |','|---|---|---|---|']
        lines += [f'| `{p}` | {s} | {r.replace("|","/")} |' for p,s,r in remaining[:15]]
    (RECORD/'清理报告.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(f'REPORT: {len(receipts)} archives, {gib(removed_bytes-archive_bytes-overhead)} GiB net, {len(remaining)} pending.')

if __name__=='__main__':
    main()
