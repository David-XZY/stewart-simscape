"""Resumable remote pipeline with two MATLAB data workers and actual GPU training."""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=['pilot', 'formal'], default='pilot')
    parser.add_argument('--run', default='pilot_v1')
    parser.add_argument('--matlab', default='D:/matlab2019a/bin/matlab.exe')
    args = parser.parse_args()
    assert args.run.replace('_', '').replace('-', '').isalnum()
    root = Path(__file__).resolve().parents[1]
    output = root/'results'/'diffusion'/args.run
    output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env['STEWART_DEPENDENCY_ROOT'] = str(root.parent/'stewart-simscape-abfd')
    env['OMP_NUM_THREADS'] = env['MKL_NUM_THREADS'] = '2'
    flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0

    def status(stage, **extra):
        temp = output/'pipeline_status.tmp'
        temp.write_text(json.dumps({'stage': stage, 'updated': time.time(), **extra}), encoding='utf-8')
        temp.replace(output/'pipeline_status.json')

    def matlab(expr, name):
        command = f"addpath('{(root/'diffusion').as_posix()}'); {expr}"
        stream = (output/f'{name}_process.log').open('a', encoding='utf-8')
        proc = subprocess.Popen([args.matlab, '-wait', '-singleCompThread', '-batch', command, '-logfile', str(output/f'{name}.log')],
                                cwd=root, env=env, stdout=stream, stderr=subprocess.STDOUT, creationflags=flags)
        stream.close()
        return proc

    def checked(expr, name):
        status(name)
        proc = matlab(expr, name)
        code = proc.wait()
        if code: raise RuntimeError(f'{name} failed with exit {code}; see {name}.log')

    try:
        # Each actual runtime must pass the model/geometry/gradient cross checks.
        checked('test_diffusion_core; test_diffusion_solver_failures', 'core_tests')
        stage1_summary = root/'results/diffusion/stage1/summary.json'
        # Revalidate nominal acceptance in the current source/runtime before
        # resuming any data; old nominal files alone are not a current gate.
        checked('run_stage1', 'stage1')
        nominal = json.loads(stage1_summary.read_text(encoding='utf-8-sig'))
        assert all(any(r['family'] == f and r['passed'] for r in nominal) for f in ('simple', 'hood')), 'Nominal gate failed'
        if not (output/'manifest.mat').exists():
            checked(f"run_dp_worker('{args.kind}',1,1,'{args.run}')", 'manifest')
        status('data_generation')
        workers = [matlab(f"run_dp_worker('data',{i},2,'{args.run}')", f'worker_{i}') for i in (1, 2)]
        while any(p.poll() is None for p in workers):
            status('data_generation', workers=[{'pid': p.pid, 'exitCode': p.poll()} for p in workers])
            time.sleep(10)
        assert all(p.returncode == 0 for p in workers), 'A data worker failed; retained checkpoints'
        checked(f"run_dp_worker('export',1,1,'{args.run}')", 'export')
        status('training')
        model_path = output/'training/model.json'
        trained = False
        if model_path.exists() and (output/'training/network_probe.json').exists():
            asset = json.loads(model_path.read_text(encoding='utf-8-sig'))
            digest = hashlib.sha256((output/'dataset.json').read_bytes()).hexdigest()
            assert asset['datasetHash'] == digest and asset['seed'] == 20260915, 'Trained asset fingerprint changed'
            trained = asset.get('status') == 'trained'
        training_command = [sys.executable, str(root/'diffusion/train_diffusion.py'), str(output/'dataset.json'),
                            str(output/'training'), '--device', 'cuda']
        if (output/'training/checkpoint.pt').exists(): training_command.append('--resume')
        if not trained:
            with (output/'training_process.log').open('a', encoding='utf-8') as stream:
                subprocess.run(training_command, check=True, env=env, stdout=stream,
                               stderr=subprocess.STDOUT, creationflags=flags)
        workers = [matlab(f"run_dp_worker('benchmark',{i},2,'{args.run}')", f'benchmark_worker_{i}') for i in (1, 2)]
        while any(p.poll() is None for p in workers):
            status('benchmark', workers=[{'pid': p.pid, 'exitCode': p.poll()} for p in workers])
            time.sleep(10)
        assert all(p.returncode == 0 for p in workers), 'A benchmark worker failed; retained task records'
        checked(f"run_dp_worker('benchmark_merge',1,1,'{args.run}')", 'benchmark_merge')
        checked(f"renderDiffusionResults('{args.run}')", 'render')
        checked(f"auditDiffusionDelivery('{args.run}')", 'audit')
        audit = json.loads((output/'delivery_audit.json').read_text(encoding='utf-8-sig'))
        status('complete', numericalDeliveryPassed=audit['numericalDeliveryPassed'], visualReviewRequired=True)
    except Exception as exc:
        status('failed', error=str(exc))
        raise


if __name__ == '__main__':
    main()
