"""Run the authorized local experiment in dependency order with durable status.

No result is inferred from a process launch. The campaign advances only after
checking the expected result files and prerequisite reports on disk.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import msvcrt
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
ART = ROOT / 'artifacts'
MATLAB = Path('D:/MATLAB/R2025b/bin/matlab.exe')
CONFIG = json.loads((ROOT / 'config.json').read_text('utf-8'))
ORDER = ['validate', 'regression', 'teacher', 'train', 'inference', 'development', 'freeze', 'formal', 'report']


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.pending')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), 'utf-8')
    os.replace(temporary, path)


def results(stage):
    records = []
    for path in (ART / stage).glob('*_force_*.json'):
        value = json.loads(path.read_text('utf-8'))
        if 'task' in value:
            assert value['version'] == CONFIG['version'], f'Incompatible result version: {path}'
            assert path.with_suffix('.mat').exists(), f'Raw record missing: {path}'
            records.append(value)
    identities = [(r['task']['id'], r['mode'], r['method'], r['seed']) for r in records]
    assert len(set(identities)) == len(identities), f'Duplicate {stage} episode'
    return records


def update(stage, state, **extra):
    counts = {}
    for name in ['teacher', 'development', 'formal']:
        values = results(name)
        counts[name] = {'records': len(values), 'completed': sum(r['status'] == 'completed' for r in values),
                        'simulation_failed': sum(r['status'] == 'simulation_failed' for r in values),
                        'setup_failed': sum(r['status'] == 'setup_failed' for r in values)}
    atomic_json(ART / 'campaign_status.json', {'version': CONFIG['version'], 'control_revision': CONFIG.get('control_revision'), 'pid': os.getpid(),
                'utc_seconds': time.time(), 'stage': stage, 'state': state, 'counts': counts, **extra})


def launch_matlab(expression, name):
    handle = (ROOT / f'{name}_console.log').open('a', encoding='utf-8')
    assert CONFIG['matlab_compute_threads'] == 1
    process = subprocess.Popen([str(MATLAB), '-singleCompThread', '-batch', "addpath('chapter5');" + expression,
        '-logfile', str(ROOT / f'{name}.log')], cwd=PROJECT, stdout=handle, stderr=subprocess.STDOUT,
        creationflags=subprocess.CREATE_NO_WINDOW)
    handle.close()
    return process


def episode_expression(stage, index, mode, seed):
    # MATLAB's Windows launcher can strip double quotes inside -batch.
    # Character literals survive argv parsing; ch5RunStage converts modes
    # to MATLAB strings before iterating, so selection semantics are intact.
    assert stage in {'development', 'formal'}
    assert mode in {'ideal-force', 'nonideal-force'}
    assert isinstance(index, int) and index >= 1
    assert isinstance(seed, int) and seed in [0, *CONFIG['training_seeds']]
    return f"ch5RunStage('{stage}',struct('indices',{index},'modes','{mode}','seeds',{seed},'worker','campaign_1'));"


def render_progress(final=False):
    completed = subprocess.run([sys.executable, str(ROOT / 'report.py'), *([] if final else ['--progress'])], cwd=PROJECT,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, creationflags=subprocess.CREATE_NO_WINDOW)
    if completed.returncode:
        raise RuntimeError(completed.stdout.decode('utf-8', errors='replace'))


def wait_processes(processes, stage):
    while True:
        codes = [p.poll() for p in processes]
        if os.name == 'nt':
            from process_resources import snapshot
            with (ART / f'{stage}_resource_usage.jsonl').open('a', encoding='utf-8') as handle:
                for p, code in zip(processes, codes):
                    if code is None:
                        sample = snapshot(p.pid);sample['control_revision'] = CONFIG.get('control_revision')
                        handle.write(json.dumps(sample) + '\n')
        update(stage, 'running', workers=[{'pid': p.pid, 'returncode': code} for p, code in zip(processes, codes)])
        if all(code is not None for code in codes):
            assert all(code == 0 for code in codes), f'{stage} worker failed; inspect its log. Return codes: {codes}'
            return
        # Do not terminate sibling runs: each owns an auditable episode and
        # finishes saving it even if another worker has encountered an error.
        render_progress()
        time.sleep(30)


def assert_complete(stage, expected):
    values = results(stage)
    assert len(values) == expected, f'{stage}: {len(values)}/{expected} explicit records'
    assert all(v['status'] in ['completed', 'simulation_failed'] for v in values), f'{stage} has unresolved setup failures'
    if stage == 'formal':
        assert sum(v['method'] == 'baseline' for v in values) == 240
        for seed in CONFIG['training_seeds']:
            assert sum(v['method'] == 'diffusion' and v['seed'] == seed for v in values) == 240


def run_python(name, stage, arguments=()):
    handle = (ROOT / f'{stage}_console.log').open('a', encoding='utf-8')
    child = subprocess.Popen([sys.executable, str(ROOT / name), *arguments], cwd=PROJECT,
        stdout=handle, stderr=subprocess.STDOUT, creationflags=subprocess.CREATE_NO_WINDOW)
    handle.close();wait_processes([child], stage)


def execute(stage, workers):
    update(stage, 'starting')
    if stage == 'validate':
        wait_processes([launch_matlab('ch5Validate;ch5CheckNumerics;ch5CheckMovingPrediction;', 'validate_campaign')], stage)
        run_python('network_probe.py', stage)
        record_environment()
    elif stage == 'regression':
        present = all((ART / 'regression' / f'strict_qp_{p}.mat').exists() for p in ['ideal_force', 'nonideal_force'])
        if not present:
            wait_processes([launch_matlab("ch5Init;run_13_strict_clf_cbf_qp_simscape('', 'chapter5/artifacts/regression',struct('sampleTime',0.01));", 'regression_campaign')], stage)
        import scipy.io
        for profile in ['ideal_force', 'nonideal_force']:
            report = scipy.io.loadmat(ART / 'regression' / f'strict_qp_{profile}.mat', simplify_cells=True)['report']
            assert report['passed']
    elif stage in ['teacher', 'development', 'formal']:
        tasks = (CONFIG['train_tasks'] + CONFIG['validation_tasks']) if stage == 'teacher' else CONFIG['development_tasks'] if stage == 'development' else 120
        expected = tasks * (2 if stage == 'teacher' else 12)
        if len(results(stage)) != expected:
            wait_processes([launch_matlab(f"ch5PrepareTasks('{stage}');ch5ExportSourceIdentity;", f'{stage}_tasks_campaign')], stage)
            expected_identity = json.loads((ART / 'runtime_source_identity.json').read_text('utf-8'))
            if stage != 'teacher':
                for record in results(stage):
                    identity = record['source_identity']
                    assert identity['baseline_source_sha256'] == expected_identity['baseline_source_sha256'], 'Cached baseline source changed; preserve results and use a new revision.'
                    if record['method'] == 'diffusion':
                        assert identity['diffusion_source_sha256'] == expected_identity['diffusion_source_sha256'], 'Cached diffusion source changed; preserve results and use a new revision.'
                # A fresh process per episode releases CasADi native graphs,
                # solver workspaces and MATLAB allocator high-water marks.
                # Cold compilation stays in each episode's timing record.
                assert workers == 1, 'Repaired control uses one bounded-lifetime MATLAB worker.'
                present = {(r['task']['index'], r['mode'], r['seed']) for r in results(stage)
                           if r['status'] in ['completed', 'simulation_failed']}
                for mode in ['ideal-force', 'nonideal-force']:
                    for seed in [0, *CONFIG['training_seeds']]:
                        for index in range(1, tasks + 1):
                            if (index, mode, seed) in present:
                                continue
                            expression = episode_expression(stage, index, mode, seed)
                            wait_processes([launch_matlab(expression, f'{stage}_campaign_1')], stage)
            else:
                processes = []
                for worker in range(workers):
                    indices = f'{worker+1}:{workers}:{tasks}'
                    expression = f"ch5RunStage('{stage}',struct('indices',{indices},'worker','campaign_{worker+1}'));"
                    processes.append(launch_matlab(expression, f'{stage}_campaign_{worker+1}'))
                wait_processes(processes, stage)
        assert_complete(stage, expected)
        if stage == 'teacher':
            wait_processes([launch_matlab('ch5ExportDataset;', 'dataset_export_campaign')], stage)
    elif stage == 'train':
        assert_complete('teacher', 400)
        run_python('train.py', stage)
    elif stage == 'inference':
        wait_processes([launch_matlab('ch5ValidateInference;', 'inference_campaign')], stage)
    elif stage == 'freeze':
        run_python('freeze.py', stage, ['--verify'] if (ART / 'freeze.json').exists() else [])
    elif stage == 'report':
        assert_complete('formal', 1440)
        run_python('freeze.py', stage, ['--verify'])
        render_progress(final=True)
        completion = json.loads((ART / 'report' / 'completion.json').read_text('utf-8'))
        assert completion['complete'], 'The final report failed its completeness audit'
    update(stage, 'completed');render_progress(final=(stage == 'report'))


def watch_existing_teachers(pids):
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.OpenProcess.argtypes = [ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    kernel.OpenProcess.restype = ctypes.c_void_p
    kernel.WaitForSingleObject.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    handles = [kernel.OpenProcess(0x00100000, False, pid) for pid in pids]
    handles = [handle for handle in handles if handle]
    try:
        while any(kernel.WaitForSingleObject(handle, 0) == 258 for handle in handles):
            update('teacher', 'waiting_for_existing_workers', existing_pids=pids)
            render_progress();time.sleep(30)
    finally:
        for handle in handles:
            kernel.CloseHandle(handle)


def record_environment():
    import torch
    interface = json.loads((ART / 'validation' / 'interface.json').read_text('utf-8'))
    compiler = subprocess.check_output(['C:/msys64/ucrt64/bin/gcc.exe', '--version'], creationflags=subprocess.CREATE_NO_WINDOW).decode('utf-8')
    import hashlib
    atomic_json(ART / 'validation' / 'environment.json', {'python': sys.version, 'python_executable': sys.executable,
        'os': platform.platform(), 'logical_processors': os.cpu_count(), 'torch': torch.__version__,
        'matlab': interface['matlab'], 'licenses': interface['licenses'], 'compiler_version': compiler.splitlines()[0],
        'compiler_sha256': hashlib.sha256(Path('C:/msys64/ucrt64/bin/gcc.exe').read_bytes()).hexdigest(),
        'workers': CONFIG['local_workers'], 'matlab_compute_threads': CONFIG['matlab_compute_threads'],
        'native_mixed_optimization': CONFIG['native_mixed_optimization']})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', choices=[*ORDER, 'all'], default='all')
    parser.add_argument('--workers', type=int, default=CONFIG['local_workers'])
    parser.add_argument('--after-teachers', nargs='*', type=int, default=[])
    parser.add_argument('--start-stage', choices=ORDER, help='Resume dependency order at an already validated stage.')
    args = parser.parse_args()
    assert 1 <= args.workers <= CONFIG['local_workers']
    ART.mkdir(parents=True, exist_ok=True)
    with (ART / 'campaign.lock').open('a+b') as lock:
        if lock.tell() == 0:
            lock.write(b'0');lock.flush()
        lock.seek(0);msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
        active_stage = args.stage
        try:
            if args.after_teachers:
                watch_existing_teachers(args.after_teachers)
            if (ART / 'validation' / 'interface.json').exists() and not (ART / 'freeze.json').exists():
                record_environment()
            stages = ORDER if args.stage == 'all' else [args.stage]
            if args.start_stage:
                assert args.stage == 'all'
                stages = ORDER[ORDER.index(args.start_stage):]
            if (ART / 'freeze.json').exists():
                # Restarting a frozen campaign must not rewrite even an
                # equivalent prerequisite MAT report with a new file header.
                subprocess.run([sys.executable, str(ROOT / 'freeze.py'), '--verify'],
                               cwd=PROJECT, check=True, creationflags=subprocess.CREATE_NO_WINDOW)
                if args.stage == 'all':
                    stages = ['formal', 'report']
                else:
                    assert args.stage in ['freeze', 'formal', 'report'], 'Frozen prerequisites are immutable; use a new experiment version.'
            for stage in stages:
                active_stage = stage
                execute(stage, args.workers)
            update('complete' if args.stage == 'all' else args.stage, 'completed')
        except BaseException as error:
            update(active_stage, 'failed', error=str(error));render_progress()
            raise
        finally:
            lock.seek(0);msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)


if __name__ == '__main__':
    main()
