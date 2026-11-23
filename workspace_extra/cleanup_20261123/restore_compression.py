"""Optional decompression of this cleanup's files; preview by default."""
import argparse,json,shutil,subprocess
from pathlib import Path
from inventory import ROOT,OUT
from compress import safe_path,sha

def candidates(prefix):
    items={}
    for name in ('probe_actions.jsonl','lzx_probe_actions.jsonl','lzx_actions.jsonl','additional_lzx_actions.jsonl','audit_storage_actions.jsonl'):
        path=OUT/name
        if not path.exists():continue
        for line in path.open(encoding='utf-8'):
            r=json.loads(line)
            if r['status'] in ('lzx_verified','compressed_verified') and r['path'].startswith(prefix):items[r['path']]=r
    # User-approved later deletion supersedes these older compression logs.
    deleted=OUT/'delete_groups_3_4_10'
    if (deleted/'verification.json').exists() and json.loads((deleted/'verification.json').read_text(encoding='utf-8'))['passed']:
        plan=json.loads((deleted/'approved_plan.json').read_text(encoding='utf-8'))
        roots=[Path(v) for v in plan['roots']];files={Path(v) for v in plan['files']}
        items={name:r for name,r in items.items() if not ((ROOT/name) in files or any((ROOT/name)==p or (ROOT/name).is_relative_to(p) for p in roots))}
    return list(items.values())

def main():
    p=argparse.ArgumentParser();p.add_argument('--prefix',default='');p.add_argument('--apply',action='store_true');args=p.parse_args();rows=candidates(args.prefix)
    print(json.dumps(dict(files=len(rows),apply=args.apply,prefix=args.prefix),ensure_ascii=False))
    if not args.apply:
        for r in rows[:25]:print(r['path'])
        return
    needed=sum(max(0,r['logical_bytes']-r['after_allocated']) for r in rows)
    if needed+1024**3>shutil.disk_usage(ROOT).free:raise RuntimeError('Insufficient free space for decompression')
    for r in rows:
        target=safe_path(r['path'])
        if sha(target)!=r['sha256']:raise RuntimeError('Changed content; refusing to alter compression: '+r['path'])
        for mode in (['/EXE'],[]):
            run=subprocess.run(['compact.exe','/U',*mode,'/I','/Q','/A',str(target)],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
            if run.returncode:raise RuntimeError(run.stdout.decode(errors='replace'))
        if sha(target)!=r['sha256']:raise RuntimeError('Decompression byte check failed')
    print('DECOMPRESSION_VERIFIED',len(rows))

if __name__=='__main__':main()
