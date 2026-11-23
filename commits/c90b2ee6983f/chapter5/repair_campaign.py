"""Sequential repair acceptance; resume the authorized experiment only on pass."""
from __future__ import annotations

import argparse
import hashlib
import json
import msvcrt
import os
import subprocess
import sys
import time
from pathlib import Path

import campaign
from process_resources import snapshot

ROOT = Path(__file__).resolve().parent
ART = ROOT / 'artifacts'
PHASES = [
    ('numerics', 'ch5CheckRepairNumerics;ch5CheckDerivativeMap;ch5CheckIntervalConstraints;ch5ExportSourceIdentity;'),
    ('narrow_nonideal', "ch5RepairProbe(2,'nonideal-force',0.32);"),
    ('inference', 'ch5ValidateInference;'),
    ('narrow_ideal', "ch5RepairProbe(2,'ideal-force',1.0);"),
    ('goal_update', "ch5RepairProbe(3,'ideal-force',0.32,'early_goal');"),
    ('moving_obstacle', "ch5RepairProbe(4,'ideal-force',0.32);"),
]
ADMISSION_BYTES = 16 * 2**30


def source_hash():
    names = sorted(p for p in ROOT.iterdir() if p.suffix in {'.m', '.py', '.json'})
    digest = hashlib.sha256()
    for p in names:
        digest.update(p.name.encode());digest.update(p.read_bytes())
    return digest.hexdigest()


def status(state, **extra):
    campaign.update('repair_validation', state, trained_models=5, **extra)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--resume-campaign', action='store_true')
    parser.add_argument('--wait-probe-pid', type=int)
    parser.add_argument('--wait-probe-report', help='Independent focused report basename, before full acceptance.')
    args = parser.parse_args()
    assert not (ART / 'freeze.json').exists(), 'An existing freeze cannot be repaired in place.'
    with (ART / 'repair.lock').open('a+b') as lock:
        if lock.tell() == 0:lock.write(b'0');lock.flush()
        lock.seek(0);msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
        expected = source_hash()
        record = {'control_revision': campaign.CONFIG['control_revision'], 'source_hash': expected,
                  'started_utc_seconds': time.time(), 'phases': [], 'passed': False,
                  'memory_admission_bytes': ADMISSION_BYTES}
        path = ART / 'validation' / 'repair_acceptance.json'
        try:
            if args.wait_probe_pid is not None:
                assert args.wait_probe_report and Path(args.wait_probe_report).name == args.wait_probe_report
                while snapshot(args.wait_probe_pid)['running']:
                    assert source_hash() == expected, 'Source changed during focused validation.'
                    status('running', phase='focused_nonideal', workers=[{'pid': args.wait_probe_pid, 'returncode': None}],
                           repairs_not_yet_validated=True)
                    time.sleep(30)
                probe = json.loads((ART / 'validation' / args.wait_probe_report).read_text('utf-8'))
                assert probe['passed'] and probe['fallbacks'] == 0, 'Focused nonideal regression failed; preserve results before another change.'
                identity = json.loads((ART / 'runtime_source_identity.json').read_text('utf-8'))
                assert probe['diffusion_source_sha256'] == identity['diffusion_source_sha256'], 'Focused probe source changed.'
            subprocess.run([sys.executable, str(ROOT / 'verify_models.py')], cwd=ROOT.parent,
                           check=True, creationflags=subprocess.CREATE_NO_WINDOW)
            for name, expression in PHASES:
                while snapshot(os.getpid())['available_commit_bytes'] < ADMISSION_BYTES:
                    status('waiting_for_memory', phase=name, required_available_commit_bytes=ADMISSION_BYTES)
                    time.sleep(30)
                assert source_hash() == expected, 'Repair source changed; preserve reports and restart validation.'
                child = campaign.launch_matlab(expression, f'repair_acceptance_{name}')
                phase = {'name': name, 'pid': child.pid, 'started_utc_seconds': time.time(),
                         'peak_tree_private_bytes': 0, 'peak_compiler_private_bytes': 0,
                         'minimum_available_commit_bytes': None}
                record['phases'].append(phase);campaign.atomic_json(path, record)
                resource = ART / 'validation' / f'repair_acceptance_{name}_memory.jsonl'
                with resource.open('a', encoding='utf-8') as handle:
                    while True:
                        sample = snapshot(child.pid)
                        handle.write(json.dumps(sample) + '\n');handle.flush()
                        phase['peak_tree_private_bytes'] = max(phase['peak_tree_private_bytes'], sample['private_bytes'])
                        phase['peak_compiler_private_bytes'] = max([phase['peak_compiler_private_bytes'], *[
                            p['private_bytes'] for p in sample['processes'] if p['name'] == 'cc1.exe']])
                        before = phase['minimum_available_commit_bytes']
                        phase['minimum_available_commit_bytes'] = sample['available_commit_bytes'] if before is None else min(before, sample['available_commit_bytes'])
                        code = child.poll()
                        status('running', phase=name, workers=[{'pid': child.pid, 'returncode': code}],
                               repairs_not_yet_validated=True)
                        campaign.atomic_json(path, record)
                        if code is not None:break
                        time.sleep(5)
                phase['returncode'] = code;phase['finished_utc_seconds'] = time.time()
                campaign.atomic_json(path, record)
                assert code == 0, f'Repair phase {name} failed; inspect its preserved MATLAB log.'
                assert phase['minimum_available_commit_bytes'] >= 2 * 2**30, f'{name}: insufficient memory headroom.'
                assert source_hash() == expected, 'Repair source changed during execution.'
            record['passed'] = True;record['finished_utc_seconds'] = time.time()
            campaign.atomic_json(path, record)
            subprocess.run([sys.executable, str(ROOT / 'repair_report.py')], cwd=ROOT.parent,
                           check=True, creationflags=subprocess.CREATE_NO_WINDOW)
            status('completed', repairs_not_yet_validated=False)
            if args.resume_campaign:
                subprocess.run([sys.executable, str(ROOT / 'campaign.py'), '--stage', 'all',
                                '--start-stage', 'development'], cwd=ROOT.parent, check=True,
                               creationflags=subprocess.CREATE_NO_WINDOW)
        except BaseException as error:
            if record['passed']:
                # A later campaign error does not invalidate completed
                # controller acceptance; campaign.py owns its stage status.
                record['post_acceptance_error'] = str(error)
                campaign.atomic_json(path, record)
            else:
                record['error'] = str(error);campaign.atomic_json(path, record)
                status('failed', error=str(error), repairs_not_yet_validated=True)
            raise
        finally:
            lock.seek(0);msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)


if __name__ == '__main__':main()
