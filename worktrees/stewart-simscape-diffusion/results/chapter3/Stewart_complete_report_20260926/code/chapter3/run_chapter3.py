"""Single-worker measured study. Resumable, explicit failures, no historical timing reuse."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def read(path):
    try:
        return json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError, ValueError):
        return None


def write(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    tmp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--run', default='chapter3_v3')
    parser.add_argument('--matlab', default='D:/matlab2019a/bin/matlab.exe')
    parser.add_argument('--report-python', default='D:/XieZhenyu/envs/stewart-chapter3-report/Scripts/python.exe')
    parser.add_argument('--stage', choices=['audit', 'select', 'prepare', 'test', 'cache-test', 'pilot', 'run'], default='run')
    args = parser.parse_args()
    assert all(c.isalnum() or c in '_-' for c in args.run)
    root = args.root.resolve(); output = root / 'results/chapter3' / args.run
    output.mkdir(parents=True, exist_ok=True)
    lock = output / 'controller.lock'
    descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    os.write(descriptor, str(os.getpid()).encode()); os.close(descriptor)
    env = dict(os.environ, OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    if Path('D:/XieZhenyu/stewart-simscape-abfd').exists():
        env['STEWART_DEPENDENCY_ROOT'] = 'D:/XieZhenyu/stewart-simscape-abfd'
    entries = {'audit': 'c3AuditLabels', 'select': 'c3SelectWeights', 'prepare': 'c3Prepare', 'test': 'test_c3_core', 'cache-test': 'test_c3_cache', 'pilot': 'c3Pilot'}
    try:
        if args.stage != 'run':
            expression = f"addpath('chapter3'); {entries[args.stage]}('{args.run}');"
            with (output / f'{args.stage}_process.log').open('a', encoding='utf-8') as log:
                result = subprocess.run([args.matlab, '-wait', '-singleCompThread', '-batch', expression],
                    cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            if result.returncode:
                raise RuntimeError(f'{args.stage} MATLAB exit {result.returncode}')
            return
        jobs = read(output / 'jobs.json'); assert jobs and read(output / 'frozen.json')
        report_python = args.report_python if Path(args.report_python).exists() else sys.executable
        subprocess.run([report_python,'-c','import numpy,matplotlib'],check=True,env=env,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        processes = 0
        while True:
            pending = [j for j in jobs if not (output / 'jobs' / j['jobId'] / 'result.json').exists()]
            write(output / 'progress.json', {'total': len(jobs), 'completed': len(jobs)-len(pending),
                'remaining': len(pending), 'workerProcesses': processes, 'updatedEpoch': time.time()})
            if not pending:
                break
            if (output / 'safety_stop.json').exists():
                raise RuntimeError('Independent acceptance mismatch; study stopped')
            expression = f"addpath('chapter3'); c3Worker('{args.run}',24);"
            active_path = output / 'active.json'; active_path.unlink(missing_ok=True)
            processes += 1
            with (output / f'worker_{processes:04d}.log').open('a', encoding='utf-8') as log:
                process = subprocess.Popen([args.matlab, '-wait', '-singleCompThread', '-batch', expression],
                    cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                killed = False
                while process.poll() is None:
                    time.sleep(1)
                    active = read(active_path)
                    if active and time.time()-active['startedEpoch'] > active['budget']+5:
                        # Own worker process tree only. No successful result is invented.
                        subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], capture_output=True,
                            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                        killed = True; break
                code = process.wait()
            active = read(active_path)
            if code or killed:
                if not active:
                    raise RuntimeError(f'Worker failed before a measured task: exit {code}')
                target = output / 'jobs' / active['job']['jobId'] / 'result.json'
                if not target.exists():
                    measured = read(target.parent / 'online_summary.json') or active['metadata']
                    row = dict(measured, completed=True, passed=False, status='timeout' if killed else 'runtime_exit',
                        processExit=code, phase=active['phase'], timingMeasured=False,
                        processObservedSeconds=time.time()-active['startedEpoch'])
                    row['timingMeasured'] = bool(measured.get('timingMeasured'))
                    progress = read(target.parent / 'progress.json')
                    if progress:
                        row['partialProgress'] = progress
                    write(target, row)
                with (output / 'runtime_incidents.jsonl').open('a', encoding='utf-8') as stream:
                    stream.write(json.dumps({'jobId': active['job']['jobId'], 'exit': code, 'killed': killed})+'\n')
            elif len(pending) == len([j for j in jobs if not (output / 'jobs' / j['jobId'] / 'result.json').exists()]):
                raise RuntimeError('Worker made no progress')
        write(output / 'complete.json', {'completed': True, 'jobCount': len(jobs), 'time': time.time()})
        # Only report after all planned calls (including failures) are accounted for.
        expression = f"addpath('chapter3'); c3ExportTrajectories('{args.run}');"
        with (output / 'export_process.log').open('a', encoding='utf-8') as log:
            subprocess.run([args.matlab, '-wait', '-singleCompThread', '-batch', expression], cwd=root, env=env,
                stdout=log, stderr=subprocess.STDOUT, check=True, creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        with (output / 'report_process.log').open('a', encoding='utf-8') as log:
            subprocess.run([report_python,str(root/'chapter3/summarize_chapter3.py'),str(output)], cwd=root, env=env,
                stdout=log,stderr=subprocess.STDOUT,check=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        write(output / 'delivery_ready.json', {'complete': True, 'report': 'report/chapter3_materials.md', 'time': time.time()})
    finally:
        lock.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
