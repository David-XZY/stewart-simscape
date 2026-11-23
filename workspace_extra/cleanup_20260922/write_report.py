from prepare_cleanup import *
import shutil,datetime,re
def main():
 v=read(OUT/'verification.json');assert v['passed']
 representatives=read(OUT/'representative_coverage.json')
 gaps=representatives['wholeFrameRetentionGaps']
 v['fullCleanupAcceptancePassed']=representatives['passed']
 v['representativeCoverageRecord']='representative_coverage.json'
 snapshot=read(OUT/'verification/snapshot_alignment/report.json');assert snapshot['passed']
 for folder in ['replay_baseline','replay_retained_case']:
  r=read(OUT/'verification'/folder/'classification.json');assert r['proofs'] and r['replays']
 archive=read(OUT/'protected_before.json')['archive'];assert digest(Path(archive['archive']))==archive['sha256']
 v['replayAndSnapshotChecksPassed']=True;v['diskFreeAfter']=shutil.disk_usage(ROOT).free
 write(OUT/'verification.json',v)
 gib=lambda n:f'{n/2**30:.3f}'
 lines=['# 工作目录清理结果',f"完成时间：{datetime.datetime.now().astimezone().isoformat()}。范围：`{ROOT}`。",
  f"删除原有冗余文件 **{gib(v['originalContentDeletedBytes'])} GiB**；另保留压缩包内独有内容 **{gib(v['newlyPreservedArchiveBytes'])} GiB**。扣除这些独有内容后的文件内容净缩减为 **{gib(v['netContentReductionBeforeAuditOverhead'])} GiB**（尚未扣除本清理记录体积）。",
  f"D 盘可用空间从扫描时 **{gib(v['diskFreeAtInventory'])} GiB** 变为 **{gib(v['diskFreeAfter'])} GiB**，实测增加 **{gib(v['diskFreeAfter']-v['diskFreeAtInventory'])} GiB**。此差值也受系统同时写入和文件系统分配影响。",
  '| 删除类别 | 文件数 | GiB |','|---|---:|---:|']
 labels={'ordinary_qp_chunk':'普通逐拍 QP 矩阵','duplicate_staging_file':'哈希相同的打包/验证副本','cache':'可重建缓存','incomplete_archive':'未完成压缩包','superseded_archive':'已有完整保留内容的旧包','retired_archive_sidecar':'旧包校验附件','verification_temporary':'本次临时解压校验文件（不计入原有空间释放）'}
 for key,size in v['categoryBytes'].items():lines.append(f"| {labels.get(key,key)} | {v['categoryCounts'][key]} | {gib(size)} |")
 lines += [f"共有 {v['skippedFiles']} 个文件因检查失败或变化而跳过。逐文件路径、删除理由、替代位置和 SHA-256 见 [deleted_files.csv](deleted_files.csv)；全部尝试见 [deletion_results.jsonl](deletion_results.jsonl)。",
  '## 验收缺口：两处极值时刻矩阵筛选遗漏',
  '本次清理未完全满足极值整拍保留要求。9 项约束极值尝试原本没有独立快照索引，其中 2 项所在时刻的其他普通矩阵被清理筛选遗漏而删除。原始轨迹和完整尝试日志仍在，但不能替代原矩阵的精确回放；现存已校验副本中未找到相同 SHA-256 文件。没有为掩盖缺口改写历史索引或重跑实验。',
  *[f"- `{x['case']}/qp_snapshots/step_{x['controlStep']:06d}.mat`：极值字段 `{x['field']}`，极值尝试 localIteration={x['localIteration']}。" for x in gaps],
  '全部 9 项原始快照缺失及 2 项清理遗漏见 [representative_coverage.json](representative_coverage.json)。以下哈希、异常覆盖、回放与缓存检查通过，不等于整体验收完全通过。',
  '## 保留与验收',
  f"当前 v8 保留 {v['retainedQpHashesPassed']} 个完整逐拍矩阵文件。本次逐拍矩阵清理涉及案例中，所有 {v['abnormalAttemptsCovered']} 次异常 QP 尝试及 {v['heldFramesCovered']} 个保持帧的已有矩阵覆盖检查通过。负的非线性预测 residual、关键误差及约束极值样本也保留。日志不完整的中断试验未因文件名推测而删除。",
  f"当前交付对应的 {v['protectedFileHashesPassed']} 个本地文件哈希未变；{v['validStatisticsCaches']} 个统计缓存仍有效。全部原始闭环轨迹、配置、噪声、统计表、失败报告与冻结控制源码保留。代表性基线和清理后的实际案例 QP 回放、快照映射检查通过；未重新运行整套闭环实验。",
  '矩阵原始完整索引和原交付清单仍保留为历史记录；[retained_indexes](retained_indexes) 是当前可用集合，带 originalIndex 和保留原因。已删除普通矩阵的历史索引条目不再表示本地可重放；全量重建需要重跑原实验。没有改写原始时序或冻结指标。',
  '旧 MPC v1–v7 工程没有整目录删除；主线、ABFD、diffusion 和扰动实验的独有源码、结果与基线保留。清理只移除了已确认重复的打包/校验副本及缓存。仍有独有内容的临时目录条目见 unique_staging_retained.json。',
  '## 保留的根目录旧包', '| 文件 | 保留原因 |','|---|---|']
 for x in read(OUT/'archive_decisions.json'):
  if x['action']=='retain' and Path(x['path']).parent==ROOT:lines.append(f"| {Path(x['path']).name} | {'含独有历史内容；展开后不节省空间，因此保留原包' if 'uniqueUncompressedBytes' in x else '独立输入/依赖或未证明存在完整替代'} |")
 lines += ['全部压缩包判定见 archive_decisions.json。删除旧包的逐成员 SHA-256 和现存路径见 archive_member_maps；其中独有内容在 retained_archive_content，文件不是丢弃后仅留哈希。',
  '## 最新交付包',f"`{archive['archive']}` 保持原样，SHA-256：`{archive['sha256']}`。最新包中“全部逐拍矩阵仍在本机”的描述属于打包时状态，现以本清理报告和保留索引为准。包旁附有同名清理说明。",
  '校验明细：[verification.json](verification.json)。删除前清单：[deletion_manifest.json](deletion_manifest.json)。']
 report_text=re.sub(r'(?m)(^\|[^\n]*\n)\n(?=\|)',r'\1','\n\n'.join(lines)+'\n')
 (OUT/'清理报告.md').write_text(report_text,encoding='utf8')
 side=Path(archive['archive']).with_suffix('.cleanup_notice.txt')
 side.write_text('该 ZIP 未修改，原 SHA-256 保持有效。\n2026-09-22 已按用户授权清理本机普通逐拍 QP 矩阵；包内清单所述全部矩阵在本机可用，仅代表原打包时状态。\n所有异常矩阵、负非线性预测 residual 样本、代表性关键样本和全部原始闭环时序保留。\n当前保留矩阵索引：'+str(OUT/'retained_indexes')+'\n完整清理记录：'+str(OUT/'清理报告.md')+'\n',encoding='utf8')
 with side.open('a',encoding='utf8') as f:f.write('验收例外：normal3_N2_A1_s101 和 normal5_N2_A1_s101 的第 246 拍普通矩阵筛选遗漏并已删除；极值整拍保留要求未完全满足。详情见清理报告。\n')
 (V8/'CLEANUP_NOTICE.txt').write_text(side.read_text(encoding='utf8'),encoding='utf8')
 print('REPORT_WRITTEN',gib(v['diskFreeAfter']-v['diskFreeAtInventory']),'GiB actual disk increase',flush=True)
if __name__=='__main__':main()
