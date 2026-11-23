"""Review list only. This script does not delete or change research files."""
import json,time
from pathlib import Path
from inventory import ROOT,OUT,allocated

U='stewart_unified_nmpc/'
GROUPS=[
 dict(id=1,name='旧扩散/IHSID 独立工程',roots=['stewart-simscape-diffusion'],note='删除该旧工程的代码、训练/推理产物和研究结果；现行统一工程及其第三章 HS2 数据不在此组。'),
 dict(id=2,name='旧第五章扩散与控制集成工程',roots=['stewart-simscape-ch5-integration'],note='删除旧 chapter5 代码、artifacts、history 和独立环境；新 ch5_adaptive 不在此组。'),
 dict(id=3,name='旧 MPC 跟踪工程系列',roots=['stewart-simscape-tracking-mpc','stewart-simscape-tracking-mpc-v2','stewart-simscape-tracking-mpc-v3','stewart-simscape-tracking-mpc-v4','stewart-simscape-tracking-mpc-v5','stewart-simscape-tracking-mpc-v6-50hz','stewart-simscape-tracking-mpc-v8-sqp-statistics'],note='删除早期 MPC/50 Hz/SQP 统计方案及其代码结果；共享 Git 元数据排除。'),
 dict(id=4,name='旧 ABFD 研究工程',roots=['stewart-simscape-abfd'],note='删除旧 ABFD 代码、计算产物和结果；该组含关联 Git 工作树，执行前需处理登记并保留共享元数据。'),
 dict(id=5,name='最早 Simscape 主工程和旧扰动对照',roots=['stewart-simscape','stewart-simscape-disturbance-experiment'],note='候选删除旧代码结果；先核对现行 SLX 外部资源。stewart-simscape/.git 是其他工作树的共享元数据，排除。'),
 dict(id=6,name='统一工程中的旧 NMPC 实验',roots=[U+k+'/'+v for k in ('results','cache') for v in ('nmpc_simple_20261116','nmpc_guidance_20261116','nmpc_rebuild_20261115','nmpc_rti_20261115')],note='旧重构/引导/简化/RTI 实验结果与缓存。保留第五章正在使用的 vendor_simd；nmpc_rti 和 nmpc_guidance 源码仍被保护校验引用，不列入整删。'),
 dict(id=7,name='上一版第四章 SOCP 跟踪实验',roots=[U+'results/ch4_tracking_20261120',U+'cache/ch4_tracking_20261120'],note='旧在线 SOCP/TVLQI 对照的大型轨迹与构建产物。references.json、原始任务时钟、校准配置等仍被当前无在线优化方案引用，不能整目录直接删除；需先分离共用小文件。'),
 dict(id=8,name='第三章早期固定/自由时长试验',roots=[U+k+'/'+v for k in ('results','cache') for v in ('ch3_ihsid_20261117','ch3_ihsid_free_time_20261118')],note='早期试验及比较数据；不包含当前 HS2 目录。删除会失去相应原始对比证据，且需先检查当前代码对旧检查点的引用。'),
 dict(id=9,name='早期统一平台与全局 NMPC 结果',roots=[U+k+'/'+v for k in ('results','cache') for v in ('long_payload_v2_20261109','smooth_nmpc_20261110','core_integration_20261112','core_repair_20261114')],note='早期实验结果和构建产物。保留当前 Python 环境，模型/物理定义/源码不在此组；代表任务和公共时标等共用文件先分离。'),
 dict(id=10,name='旧交付包与旧归档包',roots=['deliveries','_archive/20261117'],note='旧导出包及历史归档，删除会失去其中仅存的历史版本；当前第三四五章结果目录不在此组。根目录散落的旧 ZIP 另按明确名单计入。'),
]
EXCLUDED=[U+'cache/nmpc_rti_20261115/vendor_simd/',U+'cache/long_payload_v2_20261109/python/']

def main():
    rows=[json.loads(s) for s in (OUT/'inventory.jsonl').open(encoding='utf-8')];root_archives=[]
    for r in rows:
        if '/' not in r['path'] and Path(r['path']).name.lower().startswith('stewart') and Path(r['path']).suffix.lower() in ('.zip','.7z'):root_archives.append(r['path'])
    GROUPS[-1]['files']=root_archives
    groups=[dict(g,file_count=0,logical_bytes=0,current_allocated_bytes=0) for g in GROUPS]
    for r in rows:
        name=r['path']
        if '.git' in name.split('/') or any(name.startswith(v) for v in EXCLUDED):continue
        group=next((g for g in groups if any(name==p or name.startswith(p+'/') for p in g['roots']) or name in g.get('files',[])),None)
        if group is None:continue
        p=ROOT/name
        if not p.is_file():continue
        s=p.stat();size=allocated(p) if s.st_size>=1024**2 else s.st_size
        group['file_count']+=1;group['logical_bytes']+=s.st_size;group['current_allocated_bytes']+=size
    result=dict(created=time.strftime('%Y-%m-%d %H:%M:%S'),status='proposal_only_user_selection_required',groups=groups,explicit_exclusions=EXCLUDED,
                size_scope='Current approximate allocation, large files queried from NTFS; compression already in progress can further reduce these figures. Rows 7-9 are gross upper bounds before separating shared references.')
    (OUT/'deletion_options.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    lines=['# 待确认的旧方案删除清单','', '本文件仅为审批清单，没有执行新增的整组删除。体积为当前近似磁盘占用；第 7–9 组需要先分离共用引用，实际可删量小于组内总量。','',
           '| 编号 | 内容 | 当前约 GiB | 处理范围与影响 |','|---|---|---:|---|']
    for g in groups:lines.append(f"| {g['id']} | {g['name']} | {g['current_allocated_bytes']/2**30:.2f} | {g['note']} |")
    for g in groups:
        lines.extend(['',f"## {g['id']}：{g['name']}",''])
        lines.extend('- '+str(ROOT/p) for p in g['roots']+g.get('files',[]))
    (OUT/'待确认删除清单.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps([dict(id=g['id'],name=g['name'],GiB=round(g['current_allocated_bytes']/2**30,2),logical_GiB=round(g['logical_bytes']/2**30,2)) for g in groups],ensure_ascii=False,indent=2))

if __name__=='__main__':main()
