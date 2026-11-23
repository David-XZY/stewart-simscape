"""Independent post-deletion checks for exactly the approved three groups."""
import json,shutil,subprocess,time
from pathlib import Path
from inventory import ROOT,OUT
from compress import sha
from prepare_authorized_delete import DEST,git,worktrees,write

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))

def main():
    plan=read(DEST/'approved_plan.json');execution=read(DEST/'execution.json');before=read(DEST/'git_before.json')
    targets=plan['roots']+plan['files'];remaining=[p for p in targets if Path(p).exists()]
    protected=read(OUT/'protected_before.json')['files'];changed=[p for p,h in protected.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    current=worktrees();expected={row['worktree']:row for row in before['retained']};actual={row['worktree']:row for row in current}
    refs_same=git('show-ref')==before['refs'];statuses_same=True;status_differences=[]
    for path,old in before['retained_status'].items():
        value=subprocess.check_output(['git','--no-optional-locks','-C',path,'status','--porcelain=v1','--untracked-files=no'],text=True,encoding='utf-8')
        if value!=old:statuses_same=False;status_differences.append(path)
    base=ROOT/'stewart_unified_nmpc/results/ch4_noopt_20261121'
    refs=read(base/'references/uniform_bernstein_v1/registry.json')['entries'];reference_errors=[]
    for item in refs:
        r=item['selected'];p=Path(r['file'])
        if not p.is_file() or sha(p)!=r['sha256']:reference_errors.append(item['task_id'])
    designs=list((base/'designs/new_references_v1').glob('*/*/design_record.json'));design_errors=[]
    for p in designs:
        r=read(p);lib=Path(r['native_folder'])/'controller.dll'
        if not (p.parent/'design.mat').is_file() or not lib.is_file():design_errors.append(str(p.parent))
    current_package_paths=['ch5_adaptive','cache/ch5_adaptive_20261123','results/ch5_adaptive_20261123','models','tasksets','third_party','cache/long_payload_v2_20261109/python','cache/nmpc_rti_20261115/vendor_simd']
    missing_current=[p for p in current_package_paths if not (ROOT/'stewart_unified_nmpc'/p).exists()]
    unselected=read(OUT/'deletion_options.json')['groups'];unselected_missing=[]
    for group in unselected:
        if group['id'] in (3,4,10):continue
        for p in group['roots']:
            if not (ROOT/p).exists():unselected_missing.append(p)
    free=shutil.disk_usage(ROOT).free
    result=dict(created=time.strftime('%Y-%m-%d %H:%M:%S'),groups=[3,4,10],target_count=len(targets),removed_file_count=plan['file_count'],
                removed_allocated_bytes=plan['allocated_bytes'],remaining_targets=remaining,protected_file_count=len(protected),protected_changes=changed,
                reference_count=len(refs),reference_errors=reference_errors,design_count=len(designs),design_errors=design_errors,missing_current_paths=missing_current,
                removed_worktree_registrations=len(before['selected']),remaining_worktrees=len(current),remaining_worktrees_match=expected==actual,
                shared_git_present=Path(plan['protected_git_common_dir']).is_dir(),branch_refs_unchanged=refs_same,retained_tracked_status_unchanged=statuses_same,
                status_differences=status_differences,unselected_group_missing_paths=unselected_missing,disk_free_before=plan['disk_before']['free'],disk_free_after=free,
                disk_free_increase=free-plan['disk_before']['free'],backup_created=False,
                scope='Filesystem and dependency preservation checks only; no new controller simulation claimed. Shared repository object history and retained branch refs are intentionally kept.')
    result['passed']=not(remaining or changed or reference_errors or design_errors or missing_current or unselected_missing) and len(refs)==60 and len(designs)==360 and expected==actual and refs_same and statuses_same
    write('verification.json',result)
    if not result['passed']:raise RuntimeError('Deletion verification failed: '+repr(result))
    text=f'''# 第 3、4、10 组删除完成

按用户“3 4 10全部删除”的授权，已删除清单中的旧 MPC 跟踪工程系列、旧 ABFD 工程、旧交付和归档包，以及选定压缩包的校验回执。

- 删除前清单共 {plan['file_count']:,} 个文件，约 {plan['allocated_bytes']/2**30:.2f} GiB。
- 所有 {len(targets)} 个明确目标路径均已不存在。
- 移除 {len(before['selected'])} 个关联工作树登记；保留的 {len(current)} 个工作树、分支引用和已跟踪状态未变。
- 共享 Git 对象库保留；没有创建新的压缩备份。
- {len(protected)} 个保护文件哈希一致；60 条正式参考哈希一致；360 份设计及其控制器库仍在。
- 当前第三、四、五章工程和依赖保留，未选择的其他组保持原位。
- D 盘验收时可用空间：{free/2**30:.2f} GiB。

Git 初次遇到旧目录中的超长路径，后续仅对该清理命令启用了 `core.longpaths=true`，没有更改全局或仓库配置；残留路径亦已核对并清除。

[完整核验](verification.json) · [已批准范围](approved_plan.json) · [操作日志](actions.jsonl) · [删除前文件清单](manifest.jsonl)

以上为文件保护与依赖检查，不是新的控制仿真验收。磁盘空闲空间同期可能受到其他任务写入影响。
'''
    (DEST/'删除完成报告.md').write_text(text,encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('passed','groups','removed_file_count','removed_allocated_bytes','remaining_targets','protected_file_count','protected_changes','reference_count','design_count','removed_worktree_registrations','remaining_worktrees','branch_refs_unchanged','retained_tracked_status_unchanged','disk_free_after')},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
