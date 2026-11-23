"""Two-process MATLAB study supervisor; only terminates its own process trees."""
import argparse
import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import time


def read(path):
    try:
        return json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError, ValueError):
        return None


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.python.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--run', default='frozen_v1')
    parser.add_argument('--matlab', default='D:/matlab2019a/bin/matlab.exe')
    parser.add_argument('--deadline', default='2026-09-17T03:46:51+00:00')
    args = parser.parse_args()
    root = args.root.resolve()
    output = root / 'results' / 'study' / args.run
    output.mkdir(parents=True, exist_ok=True)
    deadline = dt.datetime.fromisoformat(args.deadline).timestamp()
    jobs = read(output / 'jobs.json')
    if not jobs:
        raise RuntimeError('Freeze manifest and jobs with sgPrepare before starting.')
    lock = output / 'supervisor.lock'
    # A lock must be explicitly removed after verifying the old supervisor stopped.
    lock_fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    os.write(lock_fd, str(os.getpid()).encode())
    os.close(lock_fd)
    write(output / 'run_control.json', {'computeDeadlineEpoch': deadline,
          'studyDeadlineEpoch': dt.datetime.fromisoformat('2026-09-17T09:46:51+00:00').timestamp(),
          'workers': 2, 'startEpoch': time.time(), 'supervisorPid': os.getpid()})
    workers = {}
    restarts = {1: 0, 2: 0}
    handles = []

    def launch(number):
        active = output / f'active_{number}.json'
        if active.exists():
            active.unlink()
        complete = output / f'worker_{number}_complete.json'
        if complete.exists():
            complete.unlink()
        log = open(output / f'worker_{number}.log', 'a', encoding='utf-8')
        handles.append(log)
        expression = ("setenv('STEWART_DEPENDENCY_ROOT','D:/XieZhenyu/stewart-simscape-abfd');"
                      "addpath('diffusion','study');sgWorker('%s',%d,2);" % (args.run, number))
        process = subprocess.Popen([args.matlab, '-wait', '-singleCompThread', '-batch', expression],
                                   cwd=root, stdout=log, stderr=subprocess.STDOUT,
                                   creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        workers[number] = process
        restarts[number] += 1

    def terminate(process):
        if process.poll() is None:
            subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            process.wait(timeout=30)

    def timeout_result(active, status):
        job = active['job']
        target = output / 'jobs' / job['jobId'] / 'result.json'
        if target.exists():
            return
        row = dict(active.get('metadata', {}))
        partial = read(target.parent / 'online_progress.json')
        if partial:
            row.update(partial)
        row.update(job)
        row.update({'fingerprint': active['fingerprint'], 'completed': True,
                    'passed': False, 'status': status, 'reason': status,
                    'timeoutPhase': active['phase'],
                    'elapsedPhaseSeconds': time.time() - active['startedEpoch']})
        if active['phase'] == 'online':
            row['onlineSeconds'] = row['elapsedPhaseSeconds']
        write(target, row)

    try:
        for number in (1, 2):
            launch(number)
        while workers:
            now = time.time()
            if (output / 'safety_stop.json').exists():
                raise RuntimeError('Known-model audit mismatch: study stopped; inspect safety_stop.json.')
            for number, process in list(workers.items()):
                active = read(output / f'active_{number}.json')
                if process.poll() is not None:
                    complete = read(output / f'worker_{number}_complete.json')
                    if complete or now >= deadline:
                        del workers[number]
                    elif restarts[number] >= 12:
                        raise RuntimeError(f'Worker {number} repeatedly exited; inspect its log.')
                    else:
                        if active and active.get('phase') in ('online', 'audit'):
                            timeout_result(active, 'worker_process_exit')
                        launch(number)
                    continue
                if now >= deadline:
                    terminate(process)
                    # Deadline interruption is unfinished, not a failed completed attempt.
                    del workers[number]
                    continue
                if active and active.get('phase') in ('online', 'audit'):
                    # MATLAB performs exact acceptance timing; grace is shutdown scheduling only.
                    if now - active['startedEpoch'] > active['budget'] + 3:
                        terminate(process)
                        timeout_result(active, 'wall_timeout' if active['phase'] == 'online' else 'audit_timeout')
                        launch(number)
            rows = [read(p) for p in (output / 'jobs').glob('*/result.json')]
            rows = [r for r in rows if r]
            write(output / 'progress.json', {'epoch': now, 'scheduled': len(jobs),
                  'completed': len(rows), 'passed': sum(bool(r.get('passed')) for r in rows),
                  'active': {str(n): p.pid for n, p in workers.items()}, 'launches': restarts,
                  'deadlineEpoch': deadline})
            if workers:
                time.sleep(2)
        write(output / 'supervisor_complete.json', {'epoch': time.time(), 'scheduled': len(jobs),
              'reason': 'compute_deadline' if time.time() >= deadline else 'workers_finished'})
    finally:
        for process in workers.values():
            terminate(process)
        for handle in handles:
            handle.close()
        lock.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
