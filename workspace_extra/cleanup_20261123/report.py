"""Generate a concise report from completed cleanup evidence only."""
import json
from pathlib import Path
from inventory import OUT

def main():
    v=json.loads((OUT/'verification.json').read_text(encoding='utf-8'))
    if not v['passed'] or not v['cleanup_complete']:raise RuntimeError('Cleanup completion and preservation checks are required')
    gb=lambda n:f'{n/2**30:.2f}'
    text=f'''# Matlab_code 磁盘清理记录（2026-11-23）

本轮按逐文件操作记录计算，扣除清理记录占用后，净减少约 **{gb(v['net_saved_estimate_bytes'])} GiB**。
D 盘可用空间由盘点时的 **{gb(v['disk_before']['free'])} GiB** 变为验收时的 **{gb(v['disk_after']['free'])} GiB**。
主研究线程同期仍在运行，因此全盘空闲空间变化与本轮逐文件统计可能不同。

| 处理方式 | 数量 | 减少占用 |
|---|---:|---:|
| 删除可重新生成的旧 CasADi 编译对象缓存 | {v['deleted_files']} 个文件 | {gb(v['deleted_saved_bytes'])} GiB |
| 对稳定旧文件原位无损压缩 | {v['compressed_effective_files']} 个文件 | {gb(v['compression_saved_bytes'])} GiB |

压缩对象包括自动生成的 C、旧 CSV、编译中间产物、未加载的旧库及适合压缩的旧数据。
压缩保留原路径、逻辑内容和修改时间；每个操作都核对了前后完整 SHA-256。
文件可以继续直接读取。没有设置目录继承压缩，也没有修改系统压缩策略。

## 研究内容保留核验

- {v['protected_files']} 个受保护文件哈希一致，变化 0 个。
- 60 条第四章正式参考重新核对文件哈希，全部一致。
- 360 份第四章设计文件及对应控制器库、编译回执仍完整；这些设计数据未做清理或压缩。
- 正在进行的第五章 `ch5_adaptive` 源码、缓存与结果目录全部排除。
- 当前加载的库、Python 环境、求解器依赖和 Git 元数据保留。
- 模型、原始实验结果、失败/未完成研究记录、历史独有证据保留；对旧结果仅在可压缩时无损压缩。
- 未关闭或中断任何研究进程，没有修改控制方法、参数或物理约束。

上述是文件保护与引用完整性检查，不代表重新完成控制仿真验收。
删除的对象缓存以后需要时会由现有编译入口重新生成，首次重新编译可能增加准备时间。
缺少重复证明的临时对象保留；已经密集压缩的 MAT 数据及变化/占用文件按记录跳过。

## 记录与可选解压

- [保护与引用核验](verification.json)
- [删除对象清单](deleted_objects.jsonl)
- [原位压缩记录](lzx_actions.jsonl)
- [补充压缩记录](additional_lzx_actions.jsonl)
- [原始盘点](inventory_summary.json)

无需解压即可运行。若以后希望取消本轮文件的压缩，可先预览：

```powershell
D:/Python313/python.exe -X utf8 D:/NPU/Matlab_code/cleanup_20261123/restore_compression.py --prefix "stewart_unified_nmpc/cache/ch3_ihsid_20261117/"
```

确认范围后在相同命令后加 `--apply`。工具会检查空闲空间和内容哈希，遇到已经变化的文件时停止。
它只取消压缩，不覆盖文件内容；可再生对象缓存由原项目入口重新编译。
'''
    (OUT/'清理报告.md').write_text(text,encoding='utf-8')
    print('REPORT_READY',str(OUT/'清理报告.md'))

if __name__=='__main__':main()
