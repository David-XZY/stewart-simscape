"""Inventory, archive and verify the November 2026 workspace cleanup."""
import argparse
import collections
import concurrent.futures
import csv
import hashlib
import io
import json
import os
from pathlib import Path, PureWindowsPath
import re
import shutil
import stat
import subprocess
import threading
import time
import zipfile

ROOT = Path(r'D:\NPU\Matlab_code')
RECORD = ROOT / 'cleanup_20261117'
ARCHIVE = ROOT / '_archive/20261117'
SEVEN = Path(r'C:\Program Files\7-Zip\7z.exe')
UNIFIED = 'stewart_unified_nmpc'
ONLINE = 'outputs/online_planning_control_20261014'
LIMIT = 8 * 1024**3
GIT = 'stewart-simscape'
LOCK = threading.Lock()
OLD_PROJECTS = {
    f'{GIT}-tracking-mpc-v3', f'{GIT}-tracking-mpc-v4',
    f'{GIT}-tracking-mpc-v5', f'{GIT}-tracking-mpc-v6-50hz',
    f'{GIT}-tracking-mpc-v7-estimator-qp',
    f'{GIT}-tracking-mpc-v8-sqp-statistics', ONLINE,
}
# These old probes are report evidence rather than controller inputs.
OLD_PROBES = {
    f'{GIT}-diffusion/results/chapter3/{name}' for name in (
        'boundary_callback_smoke_v1', 'boundary_cold_smoke_v1',
        'boundary_smoke_v1', 'payload_solver_smoke_v1',
        'timeout_calls_side_20260928',
    )
} | {
    f'{GIT}-abfd/results/abfd/compute/{name}' for name in (
        'A_jit_probe_20260908', 'A_inline_jacobian_isolated_20260908',
    )
}
EARLY = {
    f'{UNIFIED}/results/{name}' for name in (
        'development_partial_warm', 'nmpc_accurate_smoke',
        'nmpc_smoke', 'nmpc_smoke_sign_corrected',
    )
}
DELIVERABLES = f'{UNIFIED}/deliverables/'
STAGING = {
    'verified_extraction': 'stewart_unified_nmpc_crosscheck_20261109.zip',
    'long_payload_verified': 'stewart_long_payload_crosscheck_20261110.zip',
    'long_payload_draft_verified': 'stewart_long_payload_crosscheck_DRAFT.zip',
    'long_payload_package': 'stewart_long_payload_crosscheck_20261110.zip',
    'stewart_nmpc_snapshot_20261115_before_realtime_verify':
        'stewart_nmpc_snapshot_20261115_before_realtime.zip',
    'verify_v11_review2': 'nmpc_simple_v11_model_source_20261116_review2.zip',
    'verify_v12_review1': 'nmpc_simple_v12_model_source_20261116_review1.zip',
    'verify_v12_final': 'nmpc_simple_v12_model_source_20261116_final.zip',
}
ENV = {'.venv', 'venv', 'site-packages', 'third_party', 'vendor_simd',
       'node_modules', 'python', 'lib', 'extern', 'dependencies_snapshot'}

def sha(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()

def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))

def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')

def reparse(info):
    return bool(info.st_file_attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT)

def contained(base, rel):
    raw = PureWindowsPath(rel)
    if raw.drive or raw.root or '..' in raw.parts or not raw.parts:
        raise ValueError(f'Invalid relative path: {rel!r}')
    path = base.joinpath(*raw.parts)
    if not path.resolve().is_relative_to(base.resolve()):
        raise ValueError(f'Path escapes {base}: {path}')
    for p in (path, *path.parents):
        if p == base:
            break
        try:
            if reparse(p.lstat()):
                raise ValueError(f'Reparse point: {p}')
        except FileNotFoundError:
            continue
    return path

def same_stat(info, row):
    return (info.st_size, info.st_mtime_ns, info.st_ino, info.st_dev) == (
        row['bytes'], row['mtime_ns'], row['inode'], row['device']
    )

def git_snapshot():
    def git(tree, *args):
        return subprocess.check_output(
            ['git', '--no-optional-locks', '-C', str(tree), *args],
            stderr=subprocess.STDOUT,
        )
    raw = git(ROOT / GIT, 'worktree', 'list', '--porcelain', '-z')
    trees = [Path(item[9:]) for item in raw.decode('utf-8').split('\0')
             if item.startswith('worktree ')]
    snapshot = {'worktrees': raw.decode(), 'trees': {}}
    protected_paths = set()
    for tree in trees:
        if not tree.is_relative_to(ROOT) or not tree.is_dir():
            continue
        names = git(tree, 'ls-files', '-z').decode('utf-8').split('\0')
        status = git(tree, 'status', '--porcelain=v1', '-z', '--untracked-files=no')
        dirty_paths = git(tree, 'diff', '--name-only', '-z', 'HEAD').decode('utf-8').split('\0')
        frozen = {n: sha(tree/n) if (tree/n).is_file() else None
                  for n in dirty_paths if n}
        relative = tree.relative_to(ROOT).as_posix()
        snapshot['trees'][relative] = {
            'status': status.decode('utf-8'), 'dirty_hashes': frozen,
        }
        protected_paths.update((tree / n).relative_to(ROOT).as_posix().casefold()
                               for n in names if n)
    return snapshot, protected_paths

def protection():
    p = ROOT / UNIFIED
    smooth = read_json(p/'history/before_smooth_nmpc_20261110/baseline_manifest.json')
    simple = read_json(p/'history/before_simple_nmpc_20261116/protected_hashes.json')
    freeze = read_json(p/'results/nmpc_simple_20261116/campaigns/development_v12/source_freeze.json')
    sept = read_json(ROOT/'cleanup_20260922/protected_before.json')
    original = read_json(p/'docs/source_preservation_baseline.json')
    official = read_json(p/'results/smooth_nmpc_20261110/delivery_manifest.json')
    targets = collections.defaultdict(dict)
    def add(group, entries, anchor):
        for name, digest in entries:
            path = Path(name)
            if not path.is_absolute():
                path = anchor / path
            if path.is_relative_to(ROOT):
                targets[path.relative_to(ROOT).as_posix()][group] = digest
    def pairs(entries):
        return ((e['path'], e['sha256']) for e in entries)
    add('September', pairs(sept['files']), ROOT)
    add('September ZIP', [(sept['archive']['archive'], sept['archive']['sha256'])], ROOT)
    add('Long payload', pairs(smooth['protectedAssets']), p)
    add('Original sources', pairs(original), ROOT)
    add('Smooth source snapshot', pairs(official['entries']), p)
    add('Simple v12 protected', simple.items(), p)
    add('Simple v12 frozen sources', freeze['files'].items(), p)
    return targets

def hash_protection(targets):
    results = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        def check(item):
            relative, hashes = item
            path = ROOT / relative
            actual = sha(path) if path.is_file() else None
            return relative, {
                'sha256': actual,
                'preexisting_differences': sorted(k for k, v in hashes.items() if v != actual),
            }
        for relative, state in pool.map(check, targets.items()):
            results[relative] = state
    return results

def log(record):
    with LOCK, (RECORD/'actions.jsonl').open('a', encoding='utf-8') as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + '\n')

def run_guard():
    command = (
        "Get-CimInstance Win32_Process | Where-Object {"
        " $_.Name -match '^(matlab.*|simulink.*|gcc|cc1)\\.exe$' -or "
        "($_.Name -match '^python.*\\.exe$' -and $_.CommandLine -match 'Matlab_code|campaign\\.py|run_smooth|run_resilient')"
        " } | ForEach-Object { \"$($_.ProcessId) $($_.Name)\" }"
    )
    active = subprocess.check_output(['powershell.exe','-NoProfile','-Command',command],
                                     text=True).strip()
    if active:
        raise RuntimeError('Research process detected: ' + active)

def archive_assignment(relative, prefixes):
    for prefix in prefixes:
        if relative.startswith(prefix + '/'):
            return prefix
    return None

def read_zip_matches(folder, members, cache, digest):
    zip_path = ROOT / (DELIVERABLES + STAGING[folder])
    if not zip_path.is_file():
        return None
    if zip_path not in cache:
        z = zipfile.ZipFile(zip_path)
        zip_members = {i.filename: i for i in z.infolist() if not i.is_dir()}
        cache[zip_path] = z, zip_members
    z, zip_members = cache[zip_path]
    suffix, size = members
    if suffix not in zip_members or zip_members[suffix].file_size != size:
        return None
    with z.open(zip_members[suffix]) as handle:
        if hashlib.file_digest(handle, 'sha256').hexdigest() == digest:
            return {'path': zip_path.relative_to(ROOT).as_posix(), 'member': suffix}
    return None

def prepare():
    run_guard()
    before = shutil.disk_usage(ROOT).free
    if ROOT.resolve() != ROOT or RECORD.resolve() != RECORD or ARCHIVE.resolve() != ARCHIVE:
        raise RuntimeError('Workspace or output directory is redirected')
    git, tracked = git_snapshot()
    targets = protection()
    protected = {relative.casefold() for relative in targets}
    nested = tuple(k.casefold()+'/' for k in git['trees'] if k.count('/') > 1)
    # Confirm that each early unified result directory is absent from active source and reports.
    tokens = sorted(PureWindowsPath(p).name for p in EARLY | OLD_PROBES)
    pattern = '|'.join(re.escape(t) for t in tokens)
    checked = [UNIFIED, GIT+'-abfd', GIT+'-diffusion']
    rg = subprocess.run(
        ['rg','-l','--hidden','--no-ignore','-g','*.{m,py,ps1,md,html,json}',
         '-g','!**/.git/**','-g','!**/cache/**','-g','!**/lib/**','-g','!**/third_party/**',
         '-g','!**/history/**','-g','!**/deliverables/**','-g','!**/tasksets/**',
         '-g','!**/.venv/**','-g','!**/site-packages/**','-g','!**/results/**',
         pattern, *checked], cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
    )
    if rg.returncode not in (0, 1):
        raise RuntimeError(rg.stderr)
    references = collections.defaultdict(list)
    for name in rg.stdout.splitlines():
        source = ROOT/name
        contents = source.read_text(encoding='utf-8', errors='replace')
        for token in set(re.findall(pattern, contents, flags=0)):
            # Documentation links to linked-tree historical data are recoverable through the index.
            if source.suffix.casefold() != '.md' or name.startswith(UNIFIED):
                references[token].append(name)
    safe_probes = {p for p in EARLY | OLD_PROBES if not references[PureWindowsPath(p).name]}
    # Complete campaigns, evidence, and inputs in the unified results directory remain live.
    kept_counts = collections.Counter()
    kept_bytes = collections.Counter()
    candidates = []
    omitted = []
    historical_jobs = f'{UNIFIED}/results/jobs'
    job_root = ROOT / historical_jobs
    finished_jobs = {p.name for p in job_root.glob('job_*.m')
                     if (job_root/(p.name+'.done.json')).is_file()
                     and 'success' in read_json(job_root/(p.name+'.done.json'))}
    started = time.time_ns()
    print('Scanning workspace...', flush=True)
    for parent, dirs, files in os.walk(ROOT, followlinks=False):
        base = Path(parent)
        root_relative = base.relative_to(ROOT).as_posix()
        local_parts = base.relative_to(ROOT).parts
        for d in dirs[:]:
            path = base/d
            rel = path.relative_to(ROOT).as_posix()
            if d == '.git' or rel in {'_archive', RECORD.name}:
                dirs.remove(d)
            elif reparse(path.lstat()):
                omitted.append({'path': rel, 'reason': 'directory reparse point'})
                dirs.remove(d)
            elif any((rel.casefold()+'/').startswith(k) for k in nested):
                dirs.remove(d)
        for name in files:
            path = base/name
            relative = name if not local_parts else f'{root_relative}/{name}'
            try:
                info = path.lstat()
            except OSError as e:
                omitted.append({'path': relative, 'reason': str(e)})
                continue
            key = relative.casefold()
            parts = key.split('/')
            suffix = path.suffix.casefold()
            reason = 'no approved cleanup rule'
            action = group = None
            if reparse(info) or not stat.S_ISREG(info.st_mode):
                reason = 'non-regular file or reparse point'
            elif key in protected or key in tracked or name == '.git':
                reason = 'protected inventory or Git-tracked file'
            elif info.st_mtime_ns >= started:
                reason = 'modified during inventory'
            elif relative.startswith(('cleanup_20260922/', 'deliveries/', 'deliverables/')):
                reason = 'previous audit or formal delivery'
            else:
                env_path = bool(ENV.intersection(parts[:-1]))
                is_staging = relative.startswith(DELIVERABLES) and len(parts) >= 4 and parts[2] in STAGING
                if is_staging:
                    action, group = 'staging', 'delivery-staging'
                    reason = 'staging member review'
                elif root_relative == historical_jobs and name.removesuffix('.done.json').removesuffix('.log') in finished_jobs:
                    action, group = 'archive', 'unified-finished-jobs'
                    reason = 'finished legacy worker script with its original receipt and log'
                elif (not env_path and 'history' not in parts and
                      not relative.startswith(DELIVERABLES) and
                      re.fullmatch(r'jit_tmp[a-z0-9_]+\.c', name, flags=re.I)
                      and (not info.st_size or b'automatically generated by CasADi' in
                           path.open('rb').read(512))):
                    action, reason = 'cache', 'CasADi temporary generated C'
                elif (not env_path and 'history' not in parts and
                      not relative.startswith(DELIVERABLES) and
                      '__pycache__' in parts and suffix == '.pyc'):
                    action, reason = 'cache', 'Python interpreter bytecode'
                elif (not env_path and 'history' not in parts and
                      not relative.startswith(DELIVERABLES) and
                      (('slprj' in parts and (suffix in ('.mat','.xml','.bin','.slxc','.mldatx','.rtw',
                          '.h','.c','.obj','.o','.dll','.mexw64','.mk','.txt','.log') or name == 'tmwinternal')) or
                       (suffix == '.slxc' and ('cache' in parts or len(parts) == 1)))):
                    action, reason = 'cache', 'Simulink generated cache'
                else:
                    group = archive_assignment(relative, OLD_PROJECTS)
                    if group and not (group == ONLINE and relative.startswith(ONLINE+'/papers/')):
                        action, reason = 'archive', 'superseded independent experiment tree'
                    else:
                        group = archive_assignment(relative, safe_probes)
                        if group and not env_path:
                            action, reason = 'archive', 'historical diagnostic with no active input reference'
            if action:
                candidates.append(dict(path=relative, bytes=info.st_size, mtime_ns=info.st_mtime_ns,
                    inode=info.st_ino, device=info.st_dev, action=action, reason=reason, group=group))
            else:
                kept_counts[reason] += 1
                kept_bytes[reason] += info.st_size
    print(f'Hashing {len(candidates):,} candidates and {len(targets):,} protected paths...', flush=True)
    write_json(RECORD/'git_before.json', git)
    write_json(RECORD/'references.json', dict(found=references, archived_probe_groups=sorted(safe_probes)))
    write_json(RECORD/'protected_before.json', hash_protection(targets))
    archive_groups = collections.defaultdict(list)
    zips = {}
    planned = []
    def hash_candidate(row):
        path = ROOT/row['path']
        if not same_stat(path.lstat(), row):
            raise OSError('file changed since initial scan')
        return sha(path)
    def hash_or_skip(row):
        try:
            return hash_candidate(row), None
        except OSError as error:
            return None, str(error)
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        for row, (digest, error) in zip(candidates, pool.map(hash_or_skip, candidates)):
            if error:
                omitted.append({'path': row['path'], 'reason': error})
                continue
            row['sha256'] = digest
            if row['action'] == 'staging':
                _, _, folder, suffix = row['path'].split('/', 3)
                try:
                    identical = read_zip_matches(folder, (suffix, row['bytes']), zips, digest)
                except (OSError, zipfile.BadZipFile) as error:
                    identical = None
                    log({'event':'staging_zip_error', 'path':row['path'], 'reason':str(error)})
                if identical:
                    row.update(action='duplicate', reason='byte-identical retained ZIP member',
                               retained=identical)
                else:
                    row.update(action='archive', reason='unique or unmatched staging content')
            if row['action'] == 'archive':
                archive_groups[row['group']].append(row)
            planned.append(row)
    for z, _ in zips.values():
        z.close()
    batches = []
    for group, rows in sorted(archive_groups.items()):
        size = 0
        part = 0
        for row in sorted(rows, key=lambda r:r['path']):
            if row['bytes'] > LIMIT:
                raise RuntimeError('One file larger than archive part limit: ' + row['path'])
            if not part or size + row['bytes'] > LIMIT:
                part += 1
                size = 0
                # Flatten group names to keep path lengths low.
                name = group.removeprefix('outputs/').replace('/','__').replace('stewart_unified_nmpc__results__','unified__')
                batches.append({'name':f'{name}__{part:02d}.7z', 'files':0, 'bytes':0})
            batch = batches[-1]
            size += row['bytes']
            batch['files'] += 1
            batch['bytes'] = size
            row['archive'] = batch['name']
    with (RECORD/'manifest.jsonl').open('x', encoding='utf-8') as handle:
        for row in sorted(planned, key=lambda r:r['path']):
            handle.write(json.dumps(row, ensure_ascii=False) + '\n')
    write_json(RECORD/'batches.json', batches)
    write_json(RECORD/'inventory_notes.json', {
        'drive_before_bytes':before, 'kept_counts':kept_counts, 'kept_bytes':kept_bytes,
        'skipped':omitted, 'planned_counts':dict(collections.Counter(r['action'] for r in planned)),
        'planned_bytes':{action:sum(r['bytes'] for r in planned if r['action']==action)
                           for action in {'archive','cache','duplicate'}},
    })
    print(f'Inventory ready: {len(planned):,} files, {len(batches)} archive parts', flush=True)

def load_manifest():
    with (RECORD/'manifest.jsonl').open(encoding='utf-8') as handle:
        return [json.loads(line) for line in handle]

def command(args, cwd, logfile):
    with logfile.open('ab') as output:
        subprocess.run([str(SEVEN), *map(str,args)], cwd=cwd, stdout=output,
                       stderr=subprocess.STDOUT, check=True)

def verify_extract(archive, rows, temporary, logfile):
    temporary.mkdir()
    try:
        command(['x','-y','-bb0','-mmt=4',f'-o{temporary}',archive], ROOT, logfile)
        expected = {row['path']: row for row in rows}
        found = set()
        for path in temporary.rglob('*'):
            if reparse(path.lstat()):
                raise RuntimeError(f'Reparse point in extracted batch: {path}')
            if path.is_file():
                relative = path.relative_to(temporary).as_posix()
                row = expected[relative]
                if path.stat().st_size != row['bytes'] or sha(path) != row['sha256']:
                    raise RuntimeError(f'Extracted digest differs: {relative}')
                found.add(relative)
        if found != expected.keys():
            raise RuntimeError(f'Extraction paths differ: {set(expected)-found}')
    finally:
        # Only this process's newly created extraction directory is removed recursively.
        safe = contained(RECORD, temporary.relative_to(RECORD).as_posix())
        if safe == temporary and safe.is_dir():
            shutil.rmtree(safe)

def remove(row, duplicate_zip=None):
    try:
        path = contained(ROOT, row['path'])
        if not path.exists():
            log({'event':'missing', 'path':row['path'], 'action':row['action']})
            return
        if not same_stat(path.lstat(), row) or sha(path) != row['sha256']:
            raise OSError('contents or file identity changed since inventory')
        if duplicate_zip is not None:
            proof = row['retained']
            actual = contained(ROOT, proof['path'])
            if actual not in duplicate_zip:
                duplicate_zip[actual] = zipfile.ZipFile(actual)
            with duplicate_zip[actual].open(proof['member']) as stream:
                if hashlib.file_digest(stream,'sha256').hexdigest() != row['sha256']:
                    raise OSError('retained ZIP member changed')
        if not same_stat(path.lstat(), row):
            raise OSError('file identity changed immediately before deletion')
        path.unlink()
        log({'event':'removed', 'path':row['path'], 'action':row['action'], 'bytes':row['bytes']})
    except (OSError, ValueError, KeyError, zipfile.BadZipFile) as error:
        log({'event':'skip', 'path':row['path'], 'action':row['action'], 'reason':str(error)})

def archive_part(batch, rows):
    name = batch['name']
    archive = contained(ARCHIVE, name)
    logfile = RECORD/'logs'/f'{name}.log'
    listfile = RECORD/'lists'/f'{name}.txt'
    verification = RECORD/'verified'/f'{name}.json'
    if verification.is_file():
        receipt = read_json(verification)
        if not archive.is_file() or sha(archive) != receipt['sha256']:
            raise RuntimeError('Previously verified archive changed: ' + name)
    else:
        if archive.exists():
            raise RuntimeError('Archive exists without a verification receipt: ' + name)
        for row in rows:
            source = contained(ROOT, row['path'])
            if not same_stat(source.lstat(), row):
                raise RuntimeError('Source changed before packaging: ' + row['path'])
        listfile.write_text('\n'.join(row['path'].replace('/','\\') for row in rows)+'\n',
                            encoding='utf-8')
        partial = contained(ARCHIVE, name+'.partial')
        if partial.exists():
            raise RuntimeError('Unverified partial already exists: ' + str(partial))
        print('ARCHIVING', name, f"{batch['bytes']/1024**3:.2f} GiB", flush=True)
        method = '-m0=Deflate' if name.startswith('stewart-simscape-tracking-mpc-') else '-m0=LZMA2:d=4m'
        command(['a','-t7z','-mx=1',method,'-mmt=6','-ms=128m','-scsUTF-8',
                 '-spd','-bb0',partial,'@'+str(listfile)],ROOT,logfile)
        command(['t','-bb0','-mmt=4',partial],ROOT,logfile)
        verify_extract(partial, rows, RECORD/'extract'/name, logfile)
        digest = sha(partial)
        partial.rename(archive)
        receipt = dict(batch, sha256=digest, archive_bytes=archive.stat().st_size,
                       tested=True, extracted_sha256_verified=True)
        write_json(verification, receipt)
    run_guard()
    for row in rows:
        remove(row)
    print('VERIFIED', name, f"{receipt['archive_bytes']/1024**3:.2f} GiB compressed", flush=True)

def apply(workers):
    run_guard()
    rows = load_manifest()
    for folder in (ARCHIVE, RECORD/'logs', RECORD/'lists', RECORD/'verified', RECORD/'extract'):
        folder.mkdir(parents=True, exist_ok=True)
    removed = {e['path'] for e in read_actions() if e['event']=='removed'}
    rows = [row for row in rows if row['path'] not in removed or (ROOT/row['path']).exists()]
    direct = [row for row in rows if row['action'] in ('cache','duplicate')]
    print(f'Rechecking {len(direct):,} cache and duplicate candidates...', flush=True)
    zips = {}
    for row in direct:
        remove(row, zips if row['action']=='duplicate' else None)
    for z in zips.values():
        z.close()
    print('Cache and duplicate pass finished.', flush=True)
    groups = collections.defaultdict(list)
    for row in rows:
        if row['action'] == 'archive':
            groups[row['archive']].append(row)
    batches = [b for b in read_json(RECORD/'batches.json') if b['name'] in groups]
    errors = []
    # Use only limited parallelism to cap decompression scratch space.
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(archive_part,batch,groups[batch['name']]): batch['name']
                   for batch in batches}
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except (OSError, RuntimeError, ValueError, KeyError, subprocess.SubprocessError) as error:
                event = {'event':'batch_error', 'archive':futures[future], 'reason':str(error)}
                log(event)
                errors.append(event)
                print('BATCH FAILED', event, flush=True)
    # Remove only empty directories that were ancestors of removed manifest files.
    actions = list(read_actions())
    parents = {parent for e in actions if e['event'] == 'removed'
               for parent in PureWindowsPath(e['path']).parents if parent.parts}
    git_dirs = {i.casefold() for i in read_json(RECORD/'git_before.json')['trees']}
    for relative in sorted(parents,key=lambda p:len(p.parts),reverse=True):
        if relative.as_posix().casefold() in git_dirs or len(relative.parts) == 1 and relative.parts[0] not in OLD_PROJECTS:
            continue
        try:
            path = contained(ROOT, relative.as_posix())
            path.rmdir()
            log({'event':'empty_directory_removed','path':relative.as_posix()})
        except (OSError, ValueError):
            pass
    if errors:
        raise RuntimeError(f'{len(errors)} archive parts require review')

def read_actions():
    path = RECORD/'actions.jsonl'
    if path.exists():
        with path.open(encoding='utf-8') as handle:
            yield from map(json.loads, handle)

def validate():
    before = read_json(RECORD/'protected_before.json')
    after = hash_protection({p:{} for p in before})
    changed = [p for p in before if before[p]['sha256'] != after[p]['sha256']]
    git, _ = git_snapshot()
    receipt = {
        'protected_paths':len(before), 'changed_protected_paths':changed,
        'preexisting_protection_differences':sum(bool(r['preexisting_differences']) for r in before.values()),
        'git_worktrees_and_tracked_status_unchanged':git==read_json(RECORD/'git_before.json'),
    }
    write_json(RECORD/'protection_verification.json',receipt)
    print(json.dumps(receipt, ensure_ascii=False), flush=True)

if __name__ == '__main__':
    options = argparse.ArgumentParser()
    options.add_argument('action', choices=['prepare','apply','validate'])
    options.add_argument('--workers', type=int, choices=[1,2,3], default=2)
    args = options.parse_args()
    if args.action == 'prepare':
        prepare()
    elif args.action == 'apply':
        apply(args.workers)
    else:
        validate()
