from pathlib import Path
import csv, hashlib, json, shutil, subprocess, zipfile
from datetime import datetime

SOURCE = Path(r'C:\Users\David_X\.config\superpowers\worktrees\stewart-simscape\sc-mpc-control')
WORK = Path(__file__).resolve().parent
STAGE = WORK / 'MPC_代码与结果_20260917'
OUTPUT = Path(r'D:\NPU\Matlab_code\MPC_代码与结果_20260917.zip')

def git(*args):
    return subprocess.check_output(['git', '-C', str(SOURCE), *args]).decode('utf-8')

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def prepare():
    STAGE.mkdir(exist_ok=False)
    (STAGE / 'src').mkdir()
    selected = list(SOURCE.glob('controller/predictive_control/*.m'))
    selected += list(SOURCE.glob('controller/tests/test_5[012]_*.m'))
    selected += list(SOURCE.glob('results/reports/sampling_mpc_control/*'))
    selected += [SOURCE / p for p in [
        'controller/runs/run_10_sampling_mpc_control.m',
        'controller/simscape_tracking/computeComputedTorqueForce.m',
        'docs/controller_workflow/sampling_mpc_control_method.md', 'LICENSE']]
    selected += [SOURCE / 'opt_minimal/core' / (name + '.m') for name in [
        'buildOptModelCustom', 'computeAnchorPoints66', 'computeCompositeRequiredWrench',
        'sgpIK', 'sgpJacobian', 'rpy2rotmZYX', 'rpyRateMapZYX']]
    records = []
    for src in sorted(selected):
        relative = src.relative_to(SOURCE)
        dest = STAGE / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest)
        records.append(dict(path=relative.as_posix(), bytes=src.stat().st_size,
                            sha256=sha(src), source_mtime=datetime.fromtimestamp(src.stat().st_mtime).isoformat()))
    provenance = STAGE / 'provenance'
    provenance.mkdir()
    status = git('status', '--short')
    (WORK / 'source_snapshot.json').write_text(json.dumps(dict(records=records, status=status), indent=2), encoding='utf-8')
    (provenance / 'source_state.txt').write_text(
        f'Source: {SOURCE}\nBranch: {git("branch", "--show-current").strip()}\n'
        f'HEAD: {git("rev-parse", "HEAD").strip()}\n'
        'MPC source is untracked local work, not contained in the HEAD commit.\n'
        f'Git status before archiving:\n{status}', encoding='utf-8')
    (provenance / 'existing_test_integration.patch').write_bytes(
        subprocess.check_output(['git', '-C', str(SOURCE), 'diff', '--',
            'controller/tests/run_all_active_tests.m', 'controller/tests/test_03_directory_contract.m']))
    with (provenance / 'original_files.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=['path', 'bytes', 'sha256', 'source_mtime'])
        writer.writeheader()
        writer.writerows(records)
    print(json.dumps(dict(stage=str(STAGE), original_files=len(records),
        matlab_files=sum(r['path'].endswith('.m') for r in records),
        original_bytes=sum(r['bytes'] for r in records)), ensure_ascii=False))

def finish():
    snapshot = json.loads((WORK / 'source_snapshot.json').read_text(encoding='utf-8'))
    assert git('status', '--short') == snapshot['status'], 'Source Git status changed'
    for record in snapshot['records']:
        assert sha(SOURCE / record['path']) == record['sha256'], record['path']
        assert sha(STAGE / record['path']) == record['sha256'], record['path']
    verification = json.loads((WORK / 'verification.json').read_text(encoding='utf-8'))
    assert verification['passed'], verification
    shutil.copy2(WORK / 'verification.json', STAGE / 'provenance/verification.json')
    shutil.copy2(WORK / 'matlab_verification.log', STAGE / 'provenance/matlab_verification.log')
    files = sorted(p for p in STAGE.rglob('*') if p.is_file())
    with (STAGE / 'provenance/sha256_manifest.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(['path', 'bytes', 'sha256'])
        for path in files:
            writer.writerow([path.relative_to(STAGE).as_posix(), path.stat().st_size, sha(path)])
    assert not OUTPUT.exists(), 'Refusing to overwrite an existing archive'
    with zipfile.ZipFile(OUTPUT, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        z.writestr(STAGE.name + '/src/', '')
        for path in sorted(STAGE.rglob('*')):
            if path.is_file():
                z.write(path, f'{STAGE.name}/{path.relative_to(STAGE).as_posix()}')
    assert OUTPUT.stat().st_size < 50_000_000
    extracted = WORK / 'extraction_check'
    with zipfile.ZipFile(OUTPUT) as z:
        assert z.testzip() is None, 'ZIP CRC failure'
        for name in z.namelist():
            assert (extracted / name).resolve().is_relative_to(extracted.resolve())
        z.extractall(extracted)
        archived_files = len([x for x in z.infolist() if not x.is_dir()])
    restored = extracted / STAGE.name
    with (restored / 'provenance/sha256_manifest.csv').open(encoding='utf-8-sig', newline='') as f:
        for record in csv.DictReader(f):
            path = restored / record['path']
            assert path.stat().st_size == int(record['bytes'])
            assert sha(path) == record['sha256'], record['path']
    for record in snapshot['records']:
        assert sha(restored / record['path']) == record['sha256'], record['path']
    result = dict(archive=str(OUTPUT), bytes=OUTPUT.stat().st_size,
                  MB=OUTPUT.stat().st_size/1e6, files=archived_files,
                  sha256=sha(OUTPUT), crc='passed', extracted_hashes='passed',
                  original_source_hashes='unchanged', matlab=verification)
    (WORK / 'archive_validation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))

if __name__ == '__main__':
    import sys
    prepare() if sys.argv[1] == 'prepare' else finish()
