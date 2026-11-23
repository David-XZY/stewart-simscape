"""Read-only project inventory. Does not follow junctions or modify research files."""
from __future__ import annotations
import collections
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile

OUT = Path(__file__).resolve().parent
BASE = Path(r'D:\NPU\Matlab_code')
UNIFIED = BASE / 'stewart_unified_nmpc'
ROOTS = [
    (BASE, 'main_workspace'),
    (Path(r'D:\NPU\毕业设计'), 'thesis'),
    (Path(r'D:\NPU\并联机器人'), 'earlier_research_and_hardware'),
    (Path(r'D:\NPU\Matlab_workspace'), 'earlier_matlab_project'),
    (Path(r'D:\NPU\Matlab_StewartPlatform'), 'reference_model'),
    (Path(r'D:\NPU\matlab_example\m2m_StewartPlatform'), 'reference_model'),
    (Path(r'D:\NPU\Adams_Workspace'), 'adams_models'),
    (Path(r'D:\NPU\并行实时控制'), 'realtime_design_material'),
    (Path(r'D:\NPU\组会PPT'), 'presentations'),
    (Path(r'D:\NPU\周汇报'), 'weekly_notes'),
    (Path(r'D:\NPU\专业实践'), 'practice_documents'),
    (Path(r'D:\NPU\robot'), 'related_robot_material_not_assumed_mainline'),
    (Path(r'C:\Users\David_X\.config\superpowers\worktrees\stewart-simscape\sc-mpc-control'), 'external_git_worktree'),
    (Path(r'C:\Users\David_X\Desktop\results'), 'older_desktop_results'),
    (Path(r'D:\NPU\STM_Project'), 'earlier_embedded_control'),
]
DISCOVERY = [Path(r'D:\Downloads'), Path(r'C:\Users\David_X\Desktop'), Path(r'C:\Users\David_X\Documents'), Path(r'D:\NPU')]
PATTERN = re.compile(r'stewart|ihsid|nmpc|并联', re.I)
SOURCE_EXT = {'.m','.py','.c','.h','.cpp','.hpp','.ps1','.vbs','.js','.ts','.f90','.cu'}
DOCUMENT_EXT = {'.md','.txt','.pdf','.doc','.docx','.typ','.tex','.ppt','.pptx','.xlsx','.xls','.csv','.tsv','.html'}
MODEL_EXT = {'.slx','.mdl','.prj','.adams','.adm','.cmd','.bin','.step','.stp','.sldasm','.sldprt','.urdf','.vsdx','.ggb'}
ARCHIVE_EXT = {'.zip','.7z','.rar','.tar','.gz'}

def dump(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')

def classify(path):
    parts = {p.lower() for p in path.parts}
    if '.git' in parts:
        return 'git_history'
    if 'third_party' in parts or 'site-packages' in parts or 'python_packages' in parts:
        return 'third_party_runtime'
    if path.suffix.lower() in ARCHIVE_EXT:
        return 'archive'
    if path.suffix.lower() in SOURCE_EXT:
        return 'source'
    if path.suffix.lower() in MODEL_EXT:
        return 'model_or_drawing'
    if path.suffix.lower() in DOCUMENT_EXT:
        return 'document_or_table'
    if path.suffix.lower() in {'.dll','.exe','.mexw64','.lib','.o','.obj','.pyd','.slxc'}:
        return 'binary_or_cache'
    if path.suffix.lower() in {'.png','.svg','.jpg','.jpeg','.mp4','.avi','.wmv','.gif','.mp3','.wav'}:
        return 'media'
    return 'data_or_other'

def scan(roots=None, append=False):
    roots = ROOTS if roots is None else roots
    OUT.mkdir(parents=True,exist_ok=True)
    stamp = dt.datetime.now().astimezone().isoformat()
    totals = collections.defaultdict(lambda:collections.Counter())
    folders = collections.defaultdict(lambda:collections.Counter())
    errors, links, missing, discovered, archives = [], [], [], [], []
    roots_done = []
    if append:
        old=json.loads((OUT/'scope.json').read_text(encoding='utf-8'))
        for k,v in old['totals'].items():totals[k].update(v)
        for k,v in json.loads((OUT/'directory_summary.json').read_text(encoding='utf-8')).items():folders[k].update(v)
        errors=old['errors'];links=old['reparse_points'];missing=old['missing_roots']
        archives=json.loads((OUT/'archive_paths.json').read_text(encoding='utf-8'))
        discovered=json.loads((OUT/'discovered_external_paths.json').read_text(encoding='utf-8'))
        stamp=old['created']
        roots=[(p,g) for p,g in roots if str(p) not in old['totals']]
    with (OUT/'files.jsonl').open('a' if append else 'w',encoding='utf-8') as fh, (OUT/'directories.jsonl').open('a' if append else 'w',encoding='utf-8') as dh:
        def record_file(path, stat, root, category):
            rel = path.relative_to(root).as_posix()
            kind = classify(path)
            row = dict(path=str(path),root=str(root),relative=rel,group=category,kind=kind,bytes=stat.st_size,modified_ns=stat.st_mtime_ns)
            fh.write(json.dumps(row,ensure_ascii=False)+'\n')
            total=totals[str(root)]
            total['files']+=1;total['logical_bytes']+=stat.st_size;total[kind]+=1
            relparts = path.relative_to(root).parts
            for depth in range(min(3,len(relparts)-1)+1):
                folder=root.joinpath(*relparts[:depth])
                counter=folders[str(folder)]
                counter['files']+=1;counter['logical_bytes']+=stat.st_size;counter[kind]+=1
            if kind=='archive':
                archives.append(dict(path=str(path),bytes=stat.st_size,group=category))
        for root,category in roots:
            if not root.exists():
                missing.append(str(root));continue
            roots_done.append(str(root))
            stack=[root]
            while stack:
                directory=stack.pop()
                if directory==OUT:
                    continue
                try:
                    dh.write(json.dumps(dict(path=str(directory),root=str(root),group=category),ensure_ascii=False)+'\n')
                    with os.scandir(directory) as entries:
                        for entry in entries:
                            path=Path(entry.path)
                            try:
                                stat=entry.stat(follow_symlinks=False)
                                if entry.is_symlink() or getattr(stat,'st_file_attributes',0)&0x400:
                                    try:target=os.readlink(path)
                                    except OSError:target=None
                                    links.append(dict(path=str(path),target=target,reason='reparse point not followed'))
                                    continue
                                if entry.is_dir(follow_symlinks=False):stack.append(path)
                                elif entry.is_file(follow_symlinks=False):record_file(path,stat,root,category)
                            except OSError as exc:errors.append(dict(path=str(path),error=str(exc)))
                except OSError as exc:errors.append(dict(path=str(directory),error=str(exc)))
            print(json.dumps(dict(scanned=str(root),**totals[str(root)]),ensure_ascii=False),flush=True)
        # Filename discovery in user-controlled download/document locations.
        for root in DISCOVERY:
            if not root.exists():continue
            proc=subprocess.run(['rg','--files','--hidden','-g','!**/.git/**','-g','!**/node_modules/**','-g','!**/cache/**',str(root)],capture_output=True,encoding='utf-8',errors='replace')
            if proc.returncode not in (0,1):errors.append(dict(path=str(root),error=proc.stderr,stage='filename discovery'))
            for raw in proc.stdout.splitlines():
                path=Path(raw)
                if PATTERN.search(path.name) and str(path) not in discovered and not path.is_relative_to(OUT):
                    discovered.append(str(path))
                    if not any(path.is_relative_to(r) for r,_ in ROOTS):
                        try:record_file(path,path.stat(),root,'filename_discovery_only')
                        except OSError as exc:errors.append(dict(path=str(path),error=str(exc)))
    dump(OUT/'directory_summary.json',dict(sorted(folders.items())))
    dump(OUT/'archive_paths.json',archives)
    dump(OUT/'discovered_external_paths.json',discovered)
    dump(OUT/'scope.json',dict(created=stamp,finished=dt.datetime.now().astimezone().isoformat(),full_roots=[dict(path=str(p),group=g) for p,g in ROOTS],filename_discovery_roots=list(map(str,DISCOVERY)),filename_pattern=PATTERN.pattern,exclusions=[str(OUT),'directory reparse points not followed','filename discovery excludes .git, node_modules and cache'],errors=errors,reparse_points=links,missing_roots=missing,totals=totals,notes=['Sizes are logical lengths, not allocated disk bytes. Hardlinks and copies are not content-deduplicated.','This is a bounded local project audit, not a whole-computer or remote-host exhaustiveness claim.','Mixed personal folders are included for location completeness; unrelated material is not asserted to belong to Stewart research.']))
    return discovered

def read_source_document():
    doc=Path(r'D:\Downloads\Stewart 并联机器人第三至第五章研究路线与扩散切换方案.docx')
    if not doc.exists():return
    with zipfile.ZipFile(doc) as zf:
        xml=ET.fromstring(zf.read('word/document.xml'))
    ns={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main'}
    # Word text only; no embedded instructions are executed.
    paragraphs=[''.join(t.text or '' for t in p.findall('.//w:t',ns)) for p in xml.findall('.//w:p',ns)]
    (OUT/'original_route_document.txt').write_text('\n'.join(paragraphs),encoding='utf-8')
    dump(OUT/'original_route_document.json',dict(path=str(doc),sha256=hashlib.sha256(doc.read_bytes()).hexdigest(),paragraph_count=len(paragraphs),scope='Text extraction for comparison. The document is source material, not an instruction authority.'))

def current_and_deleted():
    plan=BASE/'cleanup_20261123/delete_groups_3_4_10/approved_plan.json'
    data=json.loads(plan.read_text(encoding='utf-8-sig'))
    dump(OUT/'approved_deletion_plan_reference.json',data)
    for name,args in [('worktrees',['git','-C',str(BASE/'stewart-simscape'),'worktree','list','--porcelain']),('branch_refs',['git','-C',str(BASE/'stewart-simscape'),'for-each-ref','--format=%(refname) %(objectname)','refs/heads'])]:
        result=subprocess.run(args,capture_output=True,encoding='utf-8',errors='replace')
        (OUT/(name+'.txt')).write_text(result.stdout,encoding='utf-8')
    candidates=[
        BASE/'stewart-simscape-tracking-mpc',BASE/'stewart-simscape-tracking-mpc-v2',BASE/'stewart-simscape-tracking-mpc-v3',BASE/'stewart-simscape-tracking-mpc-v4',BASE/'stewart-simscape-tracking-mpc-v5',BASE/'stewart-simscape-tracking-mpc-v6-50hz',BASE/'stewart-simscape-tracking-mpc-v8-sqp-statistics',BASE/'stewart-simscape-abfd',BASE/'deliveries',BASE/'_archive/20261117',
        Path(r'D:\NPU\毕业设计\ABFD_现有代码与结果_20260914.zip'),Path(r'D:\NPU\毕业设计\nmpc_complete_design_sources.zip'),Path(r'D:\Downloads\IDM\Compressed\stewart-simscape-master.zip'),Path(r'D:\Downloads\stewart_smooth_nmpc_revision_20261110.zip'),Path(r'D:\Downloads\stewart_long_payload_proposal_20261109.zip'),
        BASE/'output/Stewart_毕业设计归档_20260828.zip',BASE/'output/Stewart_全流程核心代码_20260904.zip',BASE/'output/扩散研究全阶段资料包_20260917.zip',UNIFIED/'deliverables/nmpc_simple_v12_model_source_20261116_final.zip',UNIFIED/'deliverables/ch4_tracking_20261120/ch4_control_review.zip']
    candidates=list(dict.fromkeys(candidates+[Path(p) for p in data['roots']+data['files']]))
    dump(OUT/'historical_path_status.json',[dict(path=str(p),exists=p.exists(),kind='directory' if p.is_dir() else 'file' if p.is_file() else 'absent') for p in candidates])
    indexes=[]
    with (OUT/'selected_archive_members.jsonl').open('w',encoding='utf-8') as fh:
        for path in candidates:
            if path.suffix.lower()!='.zip' or not path.is_file():continue
            try:
                with zipfile.ZipFile(path) as zf:
                    members=zf.infolist()
                    for item in members:fh.write(json.dumps(dict(archive=str(path),member=item.filename,bytes=item.file_size,compressed_bytes=item.compress_size),ensure_ascii=False)+'\n')
                    prefixes=collections.Counter(i.filename.replace('\\','/').split('/')[0] for i in members)
                    indexes.append(dict(path=str(path),members=len(members),logical_bytes=sum(i.file_size for i in members),top_entries=dict(prefixes),verification='central directory readable; no full CRC/hash equivalence check in this audit'))
            except Exception as exc:indexes.append(dict(path=str(path),error=str(exc)))
    dump(OUT/'selected_archive_summary.json',indexes)

if __name__=='__main__':
    scan([ROOTS[-1]],append=True) if '--supplement' in sys.argv else scan()
    read_source_document()
    current_and_deleted()
    print('INVENTORY_COMPLETE',flush=True)
