"""Versioned dual-fixture experiment entrypoint. No large run is implicit."""
from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import random
import re
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RESULTS = ROOT / 'results' / 'chapter3'
METHODS = ['M0', 'M1', 'M2_H', 'M3_L', 'M3_N', 'M4']
BUDGETS = [.5, 1, 2, 5, 10]


def resolve_artifact(path):
    candidate=Path(path)
    if candidate.exists(): return candidate
    normalized=str(path).replace('\\','/')
    if '/results/' in normalized:
        relocated=ROOT/'results'/normalized.split('/results/',1)[1]
        if relocated.exists(): return relocated
    return candidate


def read(path):
    return json.loads(resolve_artifact(path).read_text(encoding='utf-8-sig'))


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    temp.replace(path)


def digest(path):
    return hashlib.sha256(resolve_artifact(path).read_bytes()).hexdigest()


def canonical(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def code_manifest():
    paths = [*HERE.glob('*.m'), *HERE.glob('*.py')]
    for folder in ['chapter3','diffusion','study','opt_minimal/core','opt_minimal/ihsid','src']:
        paths.extend((ROOT / folder).glob('*.m'))
    return {str(p.relative_to(ROOT)).replace('\\', '/'): digest(p) for p in sorted(paths)}


def matlab_string(value):
    return "'" + str(value).replace('\\', '/').replace("'", "''") + "'"


def matlab(expression, timeout=None):
    binary = os.environ.get('BOUNDARY_MATLAB', 'D:/matlab2019a/bin/matlab.exe')
    if not Path(binary).is_file():
        raise RuntimeError('Set BOUNDARY_MATLAB to the MATLAB executable on the numerical host.')
    env = dict(os.environ, OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    cmd = [binary, '-wait', '-singleCompThread', '-batch', "addpath('chapter3/boundary'); " + expression]
    started = time.perf_counter()
    process = subprocess.Popen(cmd, cwd=ROOT, env=env)
    try:
        status = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        # Target only this owned MATLAB process tree; record failure at caller.
        if os.name == 'nt':
            subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], check=False, capture_output=True)
        else:
            process.kill()
        process.wait()
        raise
    if status:
        raise RuntimeError(f'MATLAB failed with exit code {status}')
    return time.perf_counter() - started


def benchmark_with_watchdog(jobs_file, run_name, cold_start=False):
    """External per-call wall watchdog. Restarts only the owned MATLAB worker.

    Failed calls stay failed; a new worker warms task-independent assets before
    continuing. Restart overhead is reported separately from resident calls.
    """
    if not re.fullmatch(r'[A-Za-z0-9_-]+',run_name):
        raise ValueError('Run name must be a simple version identifier')
    p=read(jobs_file); run_dir=RESULTS/run_name; run_dir.mkdir(parents=True,exist_ok=True)
    provenance=run_dir/'protocol_used.json'; current=dict(protocol=p,codeManifest=code_manifest())
    if provenance.exists() and read(provenance)!=current:
        raise ValueError('Run directory belongs to a different protocol/source version')
    if not provenance.exists(): write(provenance,current)
    marker=run_dir/'active_call.json'; episodes=[]
    binary=os.environ.get('BOUNDARY_MATLAB','D:/matlab2019a/bin/matlab.exe')
    env=dict(os.environ,OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    for job in p['jobs']:
        if not re.fullmatch(r'[A-Za-z0-9_.-]+',job['runId']): raise ValueError('Unsafe runId')
    for restart in range(len(p['jobs'])+1):
        remaining=[j for j in p['jobs'] if not (run_dir/'calls'/j['runId']/'result.json').exists()]
        if not remaining: break
        if marker.exists(): marker.unlink()
        expression=f"addpath('chapter3/boundary'); bdBenchmark({matlab_string(Path(jobs_file).resolve())},{matlab_string(run_name)});"
        process=subprocess.Popen([binary,'-wait','-singleCompThread','-batch',expression],cwd=ROOT,env=env,
                                 creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
        launched=time.perf_counter(); launched_unix=time.time(); killed=False
        while process.poll() is None:
            active=None
            try:
                if marker.exists(): active=read(marker)
            except (OSError,ValueError): pass
            if cold_start and not active and time.perf_counter()-launched>remaining[0]['B']+.10:
                active=dict(state='running',job=remaining[0],startedUnix=launched_unix)
            if active and active.get('state')=='running':
                job=active['job']; file=run_dir/'calls'/job['runId']/'result.json'
                elapsed=time.perf_counter()-launched if cold_start else time.time()-active['startedUnix']
                if elapsed>job['B']+.10 and not file.exists():
                    if os.name=='nt': subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,check=False)
                    else: process.kill()
                    process.wait(); killed=True
                    # Includes watchdog polling/termination overrun; never credit as success.
                    write(file,dict(taskId=job['taskId'],method=job['method'],B=job['B'],T=job['parameters'][3],job=job,
                                    passed=False,physicalPassed=False,qualityPassed=False,returnedWithinBudget=False,
                                    reason='external_wall_watchdog',codeHash=active.get('codeHash'),returnStage='',actualReturnTime=max(elapsed,job['B']),
                                    timeoutOverrun=max(0,elapsed-job['B']),independentAuditPassed=False,events=[]))
                    break
            if not active and time.perf_counter()-launched>300:
                if os.name=='nt': subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,check=False)
                else: process.kill()
                process.wait(); raise RuntimeError('Worker failed during task-independent warmup')
            time.sleep(.02)
        episodes.append(dict(restart=restart,wallSeconds=time.perf_counter()-launched,watchdogKilled=killed,exitCode=process.returncode))
        write(run_dir/'worker_episodes.json',episodes)
        if not killed and process.returncode:
            raise RuntimeError(f'Benchmark worker exited {process.returncode}; incomplete jobs retained')
        if cold_start and not killed:
            signal=read(marker); file=run_dir/'calls'/remaining[0]['runId']/'result.json'; result=read(file)
            elapsed=signal['returnedUnix']-launched_unix
            result['residentReturnTime']=result['actualReturnTime']; result['actualReturnTime']=elapsed
            result['passed']=result['passed'] and elapsed<=result['B']; result['returnedWithinBudget']=elapsed<=result['B']
            result['coldStart']=True
            if elapsed>result['B']: result['reason']='cold_start_budget_exceeded'
            write(file,result)
    return episodes


def cold_benchmark(protocol,run_name):
    """Fresh MATLAB per invocation; process/library/model/graph startup counts."""
    check_protocol(protocol); tasks={}
    for j in protocol['jobs']: tasks.setdefault(j['taskId'],j)
    chosen=sorted(tasks)[:10]
    jobs=[j for j in protocol['jobs'] if j['taskId'] in chosen and j['B'] in [2,10]]
    out=RESULTS/run_name; out.mkdir(parents=True,exist_ok=True)
    for index,job in enumerate(jobs):
        file=out/'calls'/job['runId']/'result.json'
        if file.exists(): continue
        invocation=f'{run_name}_cold_{index:04d}'; single=dict(protocol,jobs=[job]); entry=out/f'cold_job_{index:04d}.json'; write(entry,single)
        benchmark_with_watchdog(entry,invocation,cold_start=True)
        result=read(RESULTS/invocation/'calls'/job['runId']/'result.json'); result['coldStart']=True
        write(file,result)


def make_tasks(domain, seed=20260919):
    """Freeze grouped task conditions before any method is run."""
    rng = random.Random(seed)
    for key in ['a', 'b', 'psi', 'T']:
        lo, hi = domain[key]
        if not (math.isfinite(lo) and math.isfinite(hi) and lo < hi):
            raise ValueError(f'Invalid domain: {key}')
    if domain['T'][0] <= 0 or abs(domain['psi'][0] + domain['psi'][1]) > 1e-10:
        raise ValueError('Positive duration and symmetric goal-yaw range required.')
    tasks = []
    psi_max = domain['psi'][1]
    t_cut = domain['T'][0] + .25 * (domain['T'][1] - domain['T'][0])
    for i, j in itertools.product(range(10), repeat=2):
        residue = (i + 3*j) % 10
        split = 'train' if residue < 6 else 'validation' if residue < 8 else 'test_regular' if residue == 8 else 'test_combination'
        count = 5 if split == 'validation' else 10
        for k in range(count):
            a = domain['a'][0] + (i + rng.uniform(.05, .95))/10 * (domain['a'][1]-domain['a'][0])
            b = domain['b'][0] + (j + rng.uniform(.05, .95))/10 * (domain['b'][1]-domain['b'][0])
            # Disjoint conditional sampling; never reject based on planning results.
            if split == 'test_combination':
                psi = rng.choice([-1, 1]) * rng.uniform(.75*psi_max, psi_max)
                duration = rng.uniform(domain['T'][0], t_cut)
            else:
                while True:
                    psi = rng.uniform(*domain['psi']); duration = rng.uniform(*domain['T'])
                    if not (abs(psi) >= .75*psi_max and duration <= t_cut):
                        break
            row = dict(taskId=f'bd_{i:02d}_{j:02d}_{k:02d}', groupId=f'cell_{i:02d}_{j:02d}',
                       macroCellId=f'{i:02d}_{j:02d}', baseSceneId='dual_fixture_01', split=split,
                       parameters=[a, b, psi, duration])
            row['conditionHash'] = canonical(row)
            tasks.append(row)
    return dict(schemaVersion=1, seed=seed, domain=domain, tasks=tasks, hash=canonical(tasks))


def validate_tasks(manifest):
    tasks = manifest['tasks']; seen = set(); groups = {}; counts = {}
    for row in tasks:
        if row['taskId'] in seen:
            raise ValueError('Duplicate taskId')
        seen.add(row['taskId'])
        if groups.setdefault(row['groupId'], row['split']) != row['split']:
            raise ValueError('Group leakage')
        if len(row['parameters']) != 4 or not all(math.isfinite(v) for v in row['parameters']):
            raise ValueError('Invalid condition')
        counts[row['split']] = counts.get(row['split'], 0) + 1
    if manifest.get('hash') != canonical(tasks):
        raise ValueError('Task hash mismatch')
    return counts


def build_dataset(run_dir, output):
    """Keep raw teacher records; fit failures are retained outside learning rows."""
    records = []
    for split in ['train', 'validation']:
        records.extend(read(run_dir / f'{split}_reference_index.json')['records'])
    by_task = {}
    for row in records:
        if row['passed'] and row['fitPassed']:
            by_task.setdefault(row['taskId'], []).append(row)
    rows = []
    for task in sorted(by_task):
        retained = []
        for row in sorted(by_task[task], key=lambda r: r['seedId']):
            # Fixed geometry-feature RMS tolerance, disclosed as a numerical mode proxy.
            feature = row['features']
            if any(math.sqrt(sum((a-b)**2 for a,b in zip(feature,r['features']))/len(feature)) < .1 for r in retained):
                continue
            retained.append(row)
            if len(retained) == 4:
                break
        rows.extend(retained)
    if not rows or not any(r['split'] == 'validation' for r in rows):
        raise ValueError('Qualified training and validation labels are required.')
    # Train-only greedy mode prototypes; validation cannot alter the mode map.
    centers = []
    for row in sorted((r for r in rows if r['split'] == 'train'), key=lambda r:r['labelId']):
        f = row['features']
        distances = [math.sqrt(sum((a-b)**2 for a,b in zip(f,c))/len(f)) for c in centers]
        if not distances or min(distances) > 1:
            centers.append(f)
        row['modeId'] = min(range(len(centers)), key=lambda i: sum((a-b)**2 for a,b in zip(f,centers[i]))) + 1
    data = dict(kind='boundary-dataset', x=[r['context'] for r in rows], y=[r['y'] for r in rows],
                split=[r['split'] for r in rows], groups=[r['groupId'] for r in rows],
                taskIds=[r['taskId'] for r in rows], labels=rows, modeCenters=centers,
                modeDefinition='fixed scaled center/axis feature RMS; numerical motion proxy, not topology',
                manifestHash=canonical(records), codeManifest=code_manifest())
    write(output / 'dataset.json', data)
    selected_modes={r['labelId']:r.get('modeId',0) for r in rows}
    labels=[]
    for r in records:
        if r['split']=='train' and r['passed']:
            entry={k:r[k] for k in ['labelId','taskHash','split','file']}
            entry['modeId']=selected_modes.get(r['labelId'],min(range(len(centers)),key=lambda i:sum((a-b)**2 for a,b in zip(r['features'],centers[i])))+1)
            entry['fitPassed']=r['fitPassed']; entry['usableForLearning']=r['labelId'] in selected_modes
            entry['fileHash']=digest(entry['file']); labels.append(entry)
    write(output / 'library.json', dict(labels=labels, datasetHash=digest(output/'dataset.json')))
    write(output / 'label_audit.json', dict(attempts=len(records), selected=len(rows), records=records))


def quality_caps(records):
    values = [[], []]
    tasks = {}
    for r in records:
        if r['split'] != 'validation':
            raise ValueError('Quality calibration requires validation references only.')
        if r['passed']:
            tasks.setdefault(r['taskId'], []).append(r)
    if not tasks:
        raise ValueError('No qualified validation references')
    for group in tasks.values():
        for r in group:
            for i, key in enumerate(['positionDetour','rotationDetour']):
                values[i].append((r['rawQuality'][key], 1/len(tasks)/len(group)))
    out=[]
    for column in values:
        mass=0
        for value, weight in sorted(column):
            mass += weight
            if mass >= .95 - 1e-12:
                out.append(1.5*value); break
    return out


def nested_library(dataset_file, library_file, conditions, output):
    data=read(dataset_file); library=read(library_file)
    if digest(dataset_file)!=library['datasetHash']:
        raise ValueError('Dataset/library mismatch')
    task_to_group={t:g for t,g,s in zip(data['taskIds'],data['groups'],data['split']) if s=='train'}
    grouped={}
    for task,group in task_to_group.items(): grouped.setdefault(group,[]).append(task)
    for group in grouped: grouped[group].sort(key=lambda t:canonical(t))
    ordered=[]
    for level in range(max(map(len,grouped.values()))):
        for group in sorted(grouped):
            if level<len(grouped[group]): ordered.append(grouped[group][level])
    if conditions>len(ordered):
        raise ValueError(f'Only {len(ordered)} labeled training conditions exist; cannot pretend to supply {conditions}')
    selected=set(ordered[:conditions]); mask=[s=='validation' or t in selected for t,s in zip(data['taskIds'],data['split'])]
    subset=dict(data)
    for field in ['x','y','split','groups','taskIds','labels']: subset[field]=[v for v,keep in zip(data[field],mask) if keep]
    centers=[]
    for row in sorted((r for r in subset['labels'] if r['split']=='train'),key=lambda r:r['labelId']):
        f=row['features']; distances=[sum((a-b)**2 for a,b in zip(f,c))/len(f) for c in centers]
        if not distances or min(distances)>1: centers.append(f)
        row['modeId']=min(range(len(centers)),key=lambda i:sum((a-b)**2 for a,b in zip(f,centers[i])))+1
    subset['modeCenters']=centers
    subset['nestedSourceHash']=digest(dataset_file); subset['nestedConditions']=conditions
    output=Path(output); write(output/'dataset.json',subset)
    selected_labels={r['labelId'] for r in subset['labels'] if r['split']=='train'}
    selected_hashes={r['taskHash'] for r in subset['labels'] if r['split']=='train'}
    labels=[r for r in library['labels'] if r['taskHash'] in selected_hashes]
    modes={r['labelId']:r['modeId'] for r in subset['labels'] if r['split']=='train'}
    for row in labels:
        row['usableForLearning']=row['labelId'] in selected_labels
        # Raw-only rows are kept for the strong raw-path baseline, never mode regression.
        row['modeId']=modes.get(row['labelId'],0)
    write(output/'library.json',dict(labels=labels,datasetHash=digest(output/'dataset.json'),nestedSourceHash=digest(library_file),conditions=conditions))


def protocol_jobs(tasks, methods=METHODS, budgets=BUDGETS, policy='physical'):
    jobs=[]
    for task in tasks:
        for repeat in range(3):
            for B in budgets:
                order=list(methods); random.Random(f'{task["taskId"]}:{repeat}:{B}').shuffle(order)
                for method in order:
                    job=dict(task, method=method, B=B, repeat=repeat, returnPolicy=policy,
                             samplingSeed=20260919+repeat, trainingSeed=20260919+repeat)
                    job['runId']=f'{task["taskId"]}_{method}_{B:g}_{repeat}_{policy}'
                    if method in ['M3_N','M4']:
                        job['modelKey']=f'{method}_{20260919+repeat}'
                    jobs.append(job)
    return jobs


def validation_jobs(tasks,assets,phase,previous=None):
    by_group={}
    for task in sorted(tasks,key=lambda t:t['taskId']):
        if task['split']=='validation': by_group.setdefault(task['groupId'],task)
    selected=list(by_group.values()); jobs=[]
    for task in selected:
        for repeat in range(3):
            for method in METHODS:
                if phase=='methods':
                    choices=[{}]
                    if method=='M2_H': choices=[dict(distanceVariant=v) for v in ['legacy','equal_blocks','obstacle_x2','rotation_x2','time_x2']]
                    if method=='M3_L':
                        pairs=[(16,.01)]+[(n,r) for n in [8,16,32] for r in [.0001,.01,1] if (n,r)!=(16,.01)]
                        choices=[dict(localNeighbors=n,ridge=r) for n,r in pairs]
                else:
                    if previous is None: raise ValueError('Pipeline validation requires method selection first')
                    choices=[dict(previous.get(method,{}),stageFractions=f) for f in [[.5,.25,.25],[.7,.2,.1],[.4,.3,.3]]]
                for i,choice in enumerate(choices):
                    job=protocol_jobs([task],[method],[2])[repeat]
                    job.update(choice); job['configId']=f'c{i:02d}'; job['selectionPhase']=phase
                    job['runId']+=f'_{phase}_c{i:02d}'; jobs.append(job)
    random.Random(20260919).shuffle(jobs)
    return dict(assets,jobs=jobs,validationPhase=phase,previousSelection=previous or {})


def validate_results(jobs,run_dir):
    files=list((run_dir/'calls').glob('*/result.json')); calls={read(f)['job']['runId']:read(f) for f in files}
    expected={j['runId'] for j in jobs['jobs']}
    if set(calls)!=expected: raise ValueError('Validation calls are incomplete or contain unrelated runs')
    if {r['method'] for r in calls.values()}!=set(METHODS): raise ValueError('All six first-round methods must be validated')
    for r in calls.values():
        if r['job']['split']!='validation': raise ValueError('Test contamination')
        if not isinstance(r['actualReturnTime'],(int,float)) or not math.isfinite(r['actualReturnTime']): raise ValueError('Missing actual timing')
        if r['passed'] and (r['actualReturnTime']>r['B'] or not r.get('independentAuditPassed') or not r.get('fineAuditPassed',True)):
            raise ValueError('A claimed success failed independent/time audit')
    phase=jobs.get('validationPhase','fixed'); selection=dict(jobs.get('previousSelection',{})); fractions=jobs.get('stageFractions',[.5,.25,.25])
    def rate(group):
        tasks={}
        for r in group: tasks.setdefault((r['taskId'],r['method']),[]).append(float(r['passed']))
        return sum(sum(v)/len(v) for v in tasks.values())/len(tasks)
    if phase=='methods':
        for method in ['M2_H','M3_L']:
            configs={}
            for r in calls.values():
                if r['method']==method: configs.setdefault(r['job']['configId'],[]).append(r)
            chosen=sorted(configs,key=lambda k:(-rate(configs[k]),k))[0]; job=configs[chosen][0]['job']
            selection[method]={key:job[key] for key in ['distanceVariant','localNeighbors','ridge'] if key in job}
    elif phase=='pipeline':
        configs={}
        for r in calls.values(): configs.setdefault(r['job']['configId'],[]).append(r)
        chosen=sorted(configs,key=lambda k:(-rate(configs[k]),k))[0]; fractions=configs[chosen][0]['job']['stageFractions']
    return dict(passed=True,containsTestCalls=False,calls=len(calls),selection=selection,stageFractions=fractions,validationPhase=phase)


def freeze(args):
    calibration=read(args.calibration)
    if not calibration.get('mechanismEstablished'):
        raise ValueError('Stage B mechanism evidence is not established; formal freeze is blocked.')
    for evidence in calibration.get('evidenceFiles', []):
        if not Path(evidence).is_file():
            raise ValueError(f'Missing physical calibration evidence: {evidence}')
    if not calibration.get('evidenceFiles'):
        raise ValueError('Calibration must cite actual physical evidence files.')
    tasks=read(args.tasks); validate_tasks(tasks)
    validation=read(args.validation)
    if not validation.get('passed') or validation.get('containsTestCalls'):
        raise ValueError('Validation gates missing or contaminated by test calls')
    if validation.get('validationPhase')!='pipeline':
        raise ValueError('Freeze requires the completed common-pipeline validation phase')
    assets=read(args.assets)
    library=read(assets['libraryFile'])
    if library.get('purpose')=='legacy_development_diagnostic_only':
        raise ValueError('Legacy history is not the formal new-task training dataset')
    expected_models={f'{method}_{seed}' for method in ['M3_N','M4'] for seed in [20260919,20260920,20260921]}
    if {m['key'] for m in assets['models']} != expected_models:
        raise ValueError('All six declared first-round model artifacts are required')
    for entry in assets['models']:
        model=read(entry['file'])
        if model['datasetHash'] != library['datasetHash']:
            raise ValueError('Learning models and retrieval library do not share the same dataset')
    artifact_paths=[args.calibration,args.tasks,args.validation,args.assets,assets['libraryFile']]
    artifact_paths.extend(m['file'] for m in assets['models'])
    for row in library['labels']:
        if digest(row['file']) != row['fileHash']:
            raise ValueError('A historical source trajectory changed after label export')
        artifact_paths.append(row['file'])
    files={str(Path(p).resolve()):digest(p) for p in artifact_paths}
    protocol=dict(schemaVersion=1, experimentVersion='boundary_formal_v1', frozen=True,
                  codeManifest=code_manifest(), artifactHashes=files, tasksHash=tasks['hash'],
                  jobs=protocol_jobs([t for t in tasks['tasks'] if t['split'].startswith('test_')]),
                  controlCount=assets.get('controlCount',16), libraryFile=assets['libraryFile'], models=assets['models'],
                  qualityCaps=validation['qualityCaps'], stageFractions=validation.get('stageFractions',[.5,.25,.25]),
                  selection=validation.get('selection',{}), geometryScope='declared_idealized_not_CAD_verified')
    for job in protocol['jobs']: job.update(protocol['selection'].get(job['method'],{}))
    protocol['protocolHash']=canonical(protocol); write(args.output,protocol)


def check_protocol(p):
    v=dict(p); stated=v.pop('protocolHash',None)
    if not p.get('frozen') or stated != canonical(v):
        raise ValueError('Protocol is not frozen or hash is invalid')
    if p['codeManifest'] != code_manifest():
        raise ValueError('Numerical/report source changed after freeze; version and rerun affected comparisons')
    for file, expected in p['artifactHashes'].items():
        if digest(file)!=expected:
            raise ValueError(f'Frozen artifact changed: {file}')


def summarize(run_dir):
    rows=[read(p) for p in sorted((run_dir/'calls').glob('*/result.json'))]
    table=[]
    for r in rows:
        j=r['job']
        table.append(dict(taskId=r['taskId'],split=j['split'],macroCellId=j.get('macroCellId',j['groupId']),
                          method=r['method'],B=r['B'],repeat=j['repeat'],passed=r['passed'],
                          independentlyVerified=r['passed'] and r.get('independentAuditPassed',False) and r.get('fineAuditPassed',True),
                          seconds=r['actualReturnTime'],stage=r['returnStage'],reason=r['reason']))
    out=run_dir/'report'; out.mkdir(exist_ok=True)
    if table:
        with (out/'calls.csv').open('w',newline='',encoding='utf-8-sig') as f:
            writer=csv.DictWriter(f,fieldnames=list(table[0])); writer.writeheader(); writer.writerows(table)
    grouped={}
    for r in table:
        key=(r['split'],r['method'],r['B']); grouped.setdefault(key,[]).append(r)
    summary=[dict(split=k[0],method=k[1],B=k[2],calls=len(v),passed=sum(r['passed'] for r in v),
                  independentlyVerified=sum(r['independentlyVerified'] for r in v),
                  successRate=sum(r['passed'] for r in v)/len(v)) for k,v in sorted(grouped.items())]
    write(out/'summary.json',dict(calls=len(rows),groups=summary,missingCallsNotInferred=True))
    if rows:
        from plot_boundary import plot_report
        plot_report(rows,out)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__); sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('audit'); sub.add_parser('test')
    p=sub.add_parser('dev'); p.add_argument('--max-attempts',type=int,default=16); p.add_argument('--run',default='boundary_dev_v1')
    p=sub.add_parser('tasks'); p.add_argument('--calibration',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('import-legacy'); p.add_argument('--audit-file',type=Path,required=True); p.add_argument('--run',default='boundary_legacy_v1')
    p=sub.add_parser('diagnose'); p.add_argument('--tasks',type=Path,required=True); p.add_argument('--library',type=Path,required=True); p.add_argument('--max-paths',type=int,default=512); p.add_argument('--run',default='boundary_diagnostic_v1')
    p=sub.add_parser('build-data'); p.add_argument('--tasks',type=Path,required=True); p.add_argument('--split',choices=['train','validation','test_regular','test_combination'],required=True); p.add_argument('--max-attempts',type=int,required=True); p.add_argument('--run',default='boundary_data_v1'); p.add_argument('--protocol',type=Path); p.add_argument('--controls',type=int,choices=[16,20,24],default=16)
    p=sub.add_parser('dataset'); p.add_argument('--run',required=True); p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('nested-library'); p.add_argument('--dataset',type=Path,required=True); p.add_argument('--library',type=Path,required=True); p.add_argument('--conditions',type=int,choices=[150,300,600],required=True); p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('train'); p.add_argument('--dataset',type=Path,required=True); p.add_argument('--output',type=Path,required=True); p.add_argument('--method',choices=['M3_N','M4'],required=True); p.add_argument('--seed',type=int,default=20260919); p.add_argument('--device',default='cuda'); p.add_argument('--epochs',type=int,default=300)
    p=sub.add_parser('validate'); p.add_argument('--jobs',type=Path,required=True); p.add_argument('--references',type=Path,required=True); p.add_argument('--run',default='boundary_validation_v1')
    p=sub.add_parser('validation-grid'); p.add_argument('--tasks',type=Path,required=True); p.add_argument('--assets',type=Path,required=True); p.add_argument('--phase',choices=['methods','pipeline'],required=True); p.add_argument('--previous',type=Path); p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('freeze')
    for name in ['calibration','tasks','validation','assets','output']: p.add_argument('--'+name,type=Path,required=True)
    p=sub.add_parser('benchmark'); p.add_argument('--protocol',type=Path,required=True); p.add_argument('--run',default='boundary_formal_v1'); p.add_argument('--cold',action='store_true')
    p=sub.add_parser('candidates'); p.add_argument('--jobs',type=Path,required=True); p.add_argument('--run',default='boundary_candidates_v1')
    p=sub.add_parser('summarize'); p.add_argument('--run',required=True)
    args=parser.parse_args()
    if args.command=='audit':
        print(json.dumps(dict(root=str(ROOT),sourceFiles=len(code_manifest()),implementedStage='A/B core and first-round execution',largeRunsStarted=False),indent=2))
    elif args.command=='test':
        subprocess.run([sys.executable,'-m','unittest','discover','-s',str(HERE),'-p','test_boundary.py'],check=True)
        matlab('bdTests();',timeout=180)
    elif args.command=='dev':
        if not 1<=args.max_attempts<=280: raise ValueError('Stage B cap is 280 attempts')
        expression=f'bdDev({min(100,args.max_attempts)},{matlab_string(args.run)});'
        if args.max_attempts>100: expression+=f' bdStageBFollowups({matlab_string(args.run)},{args.max_attempts-100});'
        matlab(expression,timeout=args.max_attempts*150+180)
    elif args.command=='tasks':
        c=read(args.calibration)
        if not c.get('domainFrozen'): raise ValueError('Freeze the physical domain from development evidence first')
        write(args.output,make_tasks(c['domain']))
    elif args.command=='import-legacy':
        matlab(f'bdImportLegacy({matlab_string(args.audit_file.resolve())},{matlab_string(args.run)});')
    elif args.command=='diagnose':
        matlab(f'bdDiagnose({matlab_string(args.tasks.resolve())},{matlab_string(args.library.resolve())},{matlab_string(args.run)},{args.max_paths});')
    elif args.command=='build-data':
        manifest=read(args.tasks); validate_tasks(manifest)
        if args.split.startswith('test'):
            if not args.protocol: raise ValueError('Test references require frozen protocol')
            check_protocol(read(args.protocol))
        matlab(f'bdBuildData({matlab_string(args.tasks.resolve())},{matlab_string(args.split)},{matlab_string(args.run)},{args.max_attempts},{args.controls});',timeout=args.max_attempts*180+180)
    elif args.command=='dataset': build_dataset(RESULTS/args.run,args.output)
    elif args.command=='nested-library': nested_library(args.dataset,args.library,args.conditions,args.output)
    elif args.command=='train':
        from train_boundary import train
        train(args.dataset,args.output,args.method,args.seed,args.device,args.epochs)
    elif args.command=='validate':
        jobs=read(args.jobs)
        if any(j['split']!='validation' for j in jobs['jobs']): raise ValueError('Validation cannot read test tasks')
        matlab('bdTests();',timeout=180)
        benchmark_with_watchdog(args.jobs,args.run)
        summary=summarize(RESULTS/args.run)
        records=read(args.references)['records']; caps=quality_caps(records)
        result=validate_results(jobs,RESULTS/args.run); result.update(qualityCaps=caps,summary=summary)
        write(RESULTS/args.run/'validation.json',result)
    elif args.command=='validation-grid':
        tasks=read(args.tasks); validate_tasks(tasks)
        previous=read(args.previous)['selection'] if args.previous else None
        write(args.output,validation_jobs(tasks['tasks'],read(args.assets),args.phase,previous))
    elif args.command=='freeze': freeze(args)
    elif args.command=='benchmark':
        protocol=read(args.protocol); check_protocol(protocol)
        if args.cold: cold_benchmark(protocol,args.run)
        else: benchmark_with_watchdog(args.protocol,args.run)
    elif args.command=='candidates':
        matlab(f'bdCandidateDiagnostic({matlab_string(args.jobs.resolve())},{matlab_string(args.run)});')
    elif args.command=='summarize': summarize(RESULTS/args.run)


if __name__=='__main__':
    main()
