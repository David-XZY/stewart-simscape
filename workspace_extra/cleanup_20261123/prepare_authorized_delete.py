"""Freeze the user-approved groups 3, 4, 10; no deletion in this script."""
import hashlib,json,os,shutil,stat,subprocess,time
from pathlib import Path
from inventory import ROOT,OUT,allocated
from compress import sha

DEST=OUT/'delete_groups_3_4_10'
EXPECTED_ROOTS={
 'stewart-simscape-tracking-mpc','stewart-simscape-tracking-mpc-v2',
 'stewart-simscape-tracking-mpc-v3','stewart-simscape-tracking-mpc-v4',
 'stewart-simscape-tracking-mpc-v5','stewart-simscape-tracking-mpc-v6-50hz',
 'stewart-simscape-tracking-mpc-v8-sqp-statistics','stewart-simscape-abfd',
 'deliveries','_archive/20261117'}

def write(name,data):
    (DEST/name).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')

def git(*args):
    return subprocess.check_output(['git','--no-optional-locks','-C',str(ROOT/'stewart-simscape'),*args],text=True,encoding='utf-8')

def worktrees():
    raw=git('worktree','list','--porcelain');result=[]
    for block in raw.strip().split('\n\n'):
        row={}
        for line in block.splitlines():
            key,_,value=line.partition(' ');row[key]=value
        result.append(row)
    return result

def checked(rel):
    raw=Path(rel)
    if raw.is_absolute() or '..' in raw.parts:raise ValueError('Invalid relative target')
    p=ROOT/raw
    if not p.resolve().is_relative_to(ROOT) or p.resolve()==ROOT:raise ValueError('Target escapes workspace')
    for part in (p,*p.parents):
        if part==ROOT:break
        if part.exists() and part.lstat().st_file_attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT:raise ValueError('Reparse target excluded: '+str(part))
    return p

def main():
    DEST.mkdir(exist_ok=True)
    if (DEST/'approved_plan.json').exists():raise FileExistsError('Prepared deletion already exists; inspect rather than overwrite')
    proposal=json.loads((OUT/'deletion_options.json').read_text(encoding='utf-8'));groups=[g for g in proposal['groups'] if g['id'] in (3,4,10)]
    rel_roots=[r for g in groups for r in g['roots']]
    if set(rel_roots)!=EXPECTED_ROOTS:raise ValueError('Proposal scope differs from reviewed scope')
    rel_files=[r for g in groups for r in g.get('files',[])]
    # Checksum and verification sidecars belong to the explicitly approved
    # old archive packages; no similarly named research files are included.
    associated=[]
    for rel in rel_files:
        p=checked(rel)
        for candidate in (Path(str(p)+'.sha256'),Path(str(p)+'.sha256.txt'),p.with_suffix('.verification.json')):
            if candidate.is_file():associated.append(candidate.relative_to(ROOT).as_posix())
    roots=[checked(v) for v in rel_roots];files=[checked(v) for v in rel_files+associated]
    summary=[];reparse=[];errors=[];total=0;count=0;directories=[]
    with (DEST/'manifest.jsonl').open('w',encoding='utf-8') as stream:
        for target in roots+files:
            size=0;n=0;todo=[target] if target.is_dir() else []
            leaf_files=[target] if target.is_file() else []
            while todo or leaf_files:
                if todo:
                    directory=todo.pop();directories.append(directory.relative_to(ROOT).as_posix())
                    for entry in os.scandir(directory):
                        info=entry.stat(follow_symlinks=False)
                        if info.st_file_attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT:
                            reparse.append(entry.path);continue
                        if entry.is_dir(follow_symlinks=False):todo.append(Path(entry.path))
                        elif entry.is_file(follow_symlinks=False):leaf_files.append(Path(entry.path))
                while leaf_files:
                    p=leaf_files.pop();s=p.stat();allocation=allocated(p) if s.st_size>=1024**2 else s.st_size
                    record=dict(path=p.relative_to(ROOT).as_posix(),bytes=s.st_size,allocated_bytes=allocation,mtime_ns=s.st_mtime_ns,inode=s.st_ino)
                    stream.write(json.dumps(record,ensure_ascii=False)+'\n');size+=allocation;n+=1
            summary.append(dict(path=str(target),files=n,allocated_bytes=size,present=target.exists()));total+=size;count+=n
    if reparse:write('excluded_reparse_points.json',reparse);raise RuntimeError('Inspect reparse points before deletion')
    before=worktrees();selected=[];retained=[]
    for row in before:
        p=Path(row['worktree']).resolve()
        (selected if any(p==r or p.is_relative_to(r) for r in roots) else retained).append(row)
    selected.sort(key=lambda r:len(Path(r['worktree']).parts),reverse=True)
    statuses={}
    for row in retained:
        p=Path(row['worktree'])
        statuses[str(p)]=subprocess.check_output(['git','--no-optional-locks','-C',str(p),'status','--porcelain=v1','--untracked-files=no'],text=True,encoding='utf-8')
    protected=json.loads((OUT/'protected_before.json').read_text(encoding='utf-8'))['files']
    mismatches=[rel for rel,expected in protected.items() if sha(ROOT/rel)!=expected]
    if mismatches:raise RuntimeError('Protected source changed before deletion: '+repr(mismatches))
    write('git_before.json',dict(worktrees=before,selected=selected,retained=retained,retained_status=statuses,refs=git('show-ref')))
    plan=dict(created=time.strftime('%Y-%m-%d %H:%M:%S'),authorization='User: 3 4 10全部删除',groups=[3,4,10],workspace=str(ROOT),
              roots=[str(p) for p in roots],files=[str(p) for p in files],sidecars=associated,
              protected_git_common_dir=git('rev-parse','--path-format=absolute','--git-common-dir').strip(),
              worktrees_to_remove=selected,targets=summary,file_count=count,allocated_bytes=total,
              manifest_sha256=sha(DEST/'manifest.jsonl'),disk_before=dict(zip(('total','used','free'),shutil.disk_usage(ROOT))),
              policy='Delete approved old code/results/packages without making backup. Remove selected linked worktrees through Git, retaining common objects, branch refs and unselected worktrees.')
    write('approved_plan.json',plan)
    print(json.dumps(dict(targets=summary,files=count,GiB=round(total/2**30,3),worktrees_to_remove=len(selected),retained_worktrees=len(retained),protected_files=len(protected)),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
