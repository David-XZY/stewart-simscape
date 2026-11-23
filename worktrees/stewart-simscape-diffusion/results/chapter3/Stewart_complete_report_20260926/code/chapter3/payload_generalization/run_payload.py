"""Isolated, stage-gated experiment entry point. No automatic test-set tuning."""
from __future__ import annotations
import argparse
import base64
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from pg_core import VERSION,read,write,digest,development_tasks,reference_scale

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
RESULTS=ROOT/'results/chapter3'

def matlab_literal(value): return "'"+str(value).replace('\\','/').replace("'","''")+"'"

def matlab(expression,timeout=None):
    exe=os.environ.get('PG_MATLAB','D:/matlab2019a/bin/matlab.exe')
    if not Path(exe).is_file(): raise RuntimeError('Set PG_MATLAB to MATLAB with the audited CasADi/MA27 dependencies, or run on ws')
    env=os.environ.copy()
    for key in ['OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS']: env[key]='1'
    if 'STEWART_DEPENDENCY_ROOT' not in env and Path('D:/XieZhenyu/stewart-simscape-abfd').exists(): env['STEWART_DEPENDENCY_ROOT']='D:/XieZhenyu/stewart-simscape-abfd'
    command=[exe,'-wait','-singleCompThread','-batch',"addpath('chapter3/payload_generalization'); "+expression]
    return subprocess.run(command,cwd=ROOT,env=env,timeout=timeout,check=True)

def file_hash(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def stage_a_gate():
    folder=RESULTS/'payload_audit_v1'
    for name in ['matlab_audit.json','torch_audit.json']:
        if not read(folder/name).get('passed'): raise RuntimeError('Stage A audit gate failed')

def checkpoint_validator(candidates,deadline):
    # Real common MATLAB 5 ms gate, including launch time in training validation.
    folder=RESULTS/'payload_checkpoint_validation'; folder.mkdir(parents=True,exist_ok=True)
    tag=f'{time.time_ns()}'
    input_file=folder/(tag+'_input.json'); output_file=folder/(tag+'_output.json'); write(input_file,candidates)
    remaining=deadline-time.perf_counter()
    if remaining<=0: raise TimeoutError('Training deadline before checkpoint validation')
    matlab(f'pgCheckpointValidate({matlab_literal(input_file)},{matlab_literal(output_file)});',remaining)
    return read(output_file)

def frozen_hashes(protocol):
    files=[protocol['tasksFile'],protocol['datasetFile'],protocol['libraryFile']]+[m['file'] for m in protocol['models']]
    library=read(protocol['libraryFile']); root=Path(protocol['libraryFile']).parent
    files.extend(str(root/r['file']) for r in library['records'])
    if protocol.get('integrationAuditFile'): files.append(protocol['integrationAuditFile'])
    return {str(Path(f).resolve()):file_hash(f) for f in files}

def freeze(request,output):
    stage_a_gate(); p=read(request)
    if p.get('experimentVersion')!=VERSION: raise ValueError('Wrong experiment version')
    integration=p.get('integrationAuditFile',str(RESULTS/'payload_integration_v1/integration.json'))
    if not read(integration).get('passed'): raise ValueError('Integration tests not passed')
    from pg_learning import validate_dataset
    validate_dataset(read(p['datasetFile']))
    required=['matlabCodeHash','controlCount','models','jobs','qualityCaps','validationSelection','tasksFile','datasetFile','libraryFile']
    for key in required:
        if key not in p: raise ValueError('Missing frozen field: '+key)
    if not p['jobs'] or not p['models']: raise ValueError('Empty formal protocol')
    if p['validationSelection'].get('split')!='validation': raise ValueError('Selection must only use validation')
    for model in p['models']:
        asset=read(model['file'])
        if asset.get('status')!='trained' or asset.get('contextSchema') not in ['pg_basic99_v1','pg_signature354_v1']: raise ValueError('Untrained or legacy model')
        if asset.get('datasetHash')!=file_hash(p['datasetFile']): raise ValueError('Models must share this exact source dataset')
    manifest=read(p['tasksFile']); ids={t['taskId']:t for t in manifest['tasks']}
    for j in p['jobs']:
        if j['spec']['taskId'] not in ids or j['spec']!=ids[j['spec']['taskId']]: raise ValueError('Job task differs from frozen manifest')
        if j['B'] not in [.5,1,2,5]: raise ValueError('Unknown online budget')
    p.update(status='frozen',fileHashes=frozen_hashes(p)); p['configHash']=digest(p)
    if Path(output).exists(): raise ValueError('Immutable frozen protocol exists')
    write(output,p); return p

def check_frozen(path):
    p=read(path)
    if p.get('status')!='frozen': raise ValueError('Protocol not frozen')
    hashed=dict(p); expected=hashed.pop('configHash')
    if digest(hashed)!=expected: raise ValueError('Protocol hash mismatch')
    if frozen_hashes(p)!=p['fileHashes']: raise ValueError('Task/data/library/model changed after freeze')
    return p

def main():
    parser=argparse.ArgumentParser(description=__doc__); sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('audit')
    p=sub.add_parser('make-tasks'); p.add_argument('--output',default=str(RESULTS/'payload_tasks_v1/development_manifest.json')); p.add_argument('--formal-size',type=int,choices=[150,300,600]); p.add_argument('--resource-run',default='payload_references_v1')
    for name in ['pilot','references']:
        p=sub.add_parser(name); p.add_argument('--manifest',default=str(RESULTS/'payload_tasks_v1/development_manifest.json')); p.add_argument('--run',default='payload_'+('pilot' if name=='pilot' else 'references')+'_v1')
        if name=='references': p.add_argument('--max-attempts',type=int,default=576)
    p=sub.add_parser('build-dataset'); p.add_argument('--sources',nargs='+',default=['payload_pilot_v1','payload_references_v1']); p.add_argument('--run',default='payload_dataset_v1')
    p=sub.add_parser('train'); p.add_argument('--method',required=True); p.add_argument('--dataset',required=True); p.add_argument('--output',required=True); p.add_argument('--seed',type=int,default=20260919); p.add_argument('--device',default='cuda'); p.add_argument('--physics-weight',type=float,default=.1); p.add_argument('--config',default=str(HERE/'protocol.json')); p.add_argument('--resume',action='store_true')
    p=sub.add_parser('validate'); p.add_argument('--input',required=True); p.add_argument('--output',required=True)
    p=sub.add_parser('select-retrieval'); p.add_argument('--library',required=True); p.add_argument('--manifest',required=True); p.add_argument('--run',default='payload_validation_selection_v1')
    p=sub.add_parser('diagnose-library'); p.add_argument('--library',required=True); p.add_argument('--tasks',required=True); p.add_argument('--run',default='payload_library_diagnostic_v1'); p.add_argument('--max-paths',type=int,default=512)
    p=sub.add_parser('freeze'); p.add_argument('--request',required=True); p.add_argument('--output',required=True)
    p=sub.add_parser('benchmark'); p.add_argument('--protocol',required=True); p.add_argument('--run',required=True)
    p=sub.add_parser('summarize'); p.add_argument('--run',required=True)
    p=sub.add_parser('resource-estimate'); p.add_argument('--run',default='payload_pilot_v1')
    a=parser.parse_args()
    if a.command=='make-tasks':
        if a.formal_size:
            resource=read(RESULTS/a.resource_run/'resource_estimate.json')
            if not resource.get('coverageAdequateForSizeSelection') or resource.get('formalTrainSizeWithin72h') is None or a.formal_size>resource['formalTrainSizeWithin72h']: raise ValueError('Formal reference resource/coverage gate not met')
            from pg_formal_tasks import formal_tasks
            content=formal_tasks(a.formal_size)
        else: content=development_tasks()
        if Path(a.output).exists() and read(a.output)!=content: raise ValueError('Existing task manifest differs')
        write(a.output,content); print(a.output)
    elif a.command=='audit':
        subprocess.run([sys.executable,'-m','unittest','discover','-s',str(HERE),'-p','test_payload.py','-v'],check=True)
        matlab('pgAudit; pgIntegrationTests;')
        from audit_torch import audit
        audit(RESULTS/'payload_audit_v1/physics_oracle.json',RESULTS/'payload_audit_v1/torch_audit.json')
        from audit_penalty import audit as penalty_audit
        penalty_audit(RESULTS/'payload_integration_v1/penalty_oracle.json',RESULTS/'payload_integration_v1/penalty_audit.json')
    elif a.command=='pilot':
        stage_a_gate(); matlab(f'pgPilot({matlab_literal(a.manifest)},{matlab_literal(a.run)});',32*125+300)
    elif a.command=='references':
        stage_a_gate(); matlab(f'pgReferenceBatch({matlab_literal(a.manifest)},{matlab_literal(a.run)},{a.max_attempts});',a.max_attempts*125+600)
    elif a.command=='build-dataset':
        names='{'+','.join(matlab_literal(s) for s in a.sources)+'}'
        matlab(f'pgBuildDataset({names},{matlab_literal(a.run)});')
    elif a.command=='train':
        stage_a_gate(); from pg_learning import train
        train(a.dataset,a.output,a.method,a.seed,checkpoint_validator,read(a.config),device=a.device,physics_weight=a.physics_weight,resume=a.resume)
    elif a.command=='validate': matlab(f'pgCheckpointValidate({matlab_literal(a.input)},{matlab_literal(a.output)});')
    elif a.command=='select-retrieval': matlab(f'pgValidationSelect({matlab_literal(a.library)},{matlab_literal(a.manifest)},{matlab_literal(a.run)});')
    elif a.command=='diagnose-library': matlab(f'pgDiagnoseLibrary({matlab_literal(a.library)},{matlab_literal(a.tasks)},{matlab_literal(a.run)},{a.max_paths});',24*3600+300)
    elif a.command=='freeze': freeze(a.request,a.output)
    elif a.command=='benchmark':
        check_frozen(a.protocol); matlab(f'pgBenchmark({matlab_literal(a.protocol)},{matlab_literal(a.run)});')
    elif a.command=='summarize':
        from report_payload_generalization import summarize
        print(summarize(RESULTS/a.run))
    elif a.command=='resource-estimate':
        attempts=[read(p) for p in sorted((RESULTS/a.run).glob('attempt_*.json'))]
        # Include failed solver calls; separately report zero-solve endpoint rejects.
        called=[r['actualAttemptSeconds'] for r in attempts if r.get('endpointPassed')]
        if not called: raise ValueError('No actual solver timings')
        import numpy as np
        frozen=read(RESULTS/a.run/'frozen_tasks.json'); specs={t['taskId']:t for t in frozen['tasks']}; observed=[specs[r['taskId']] for r in attempts if r.get('endpointPassed')]
        classes=sorted({r['taskClass'] for r in observed}); families=sorted({r['family'] for r in observed}); shapes=sorted({r['shape'] for r in observed})
        adequate=len(classes)==3 and len(families)==2 and len(shapes)==2
        avg=float(np.mean(called)); estimate={'solverCalls':len(called),'endpointRejects':len(attempts)-len(called),'meanAttemptSeconds':avg,'p50':float(np.median(called)),'p95':float(np.quantile(called,.95)),
                  'formalTrainSizeWithin72h':reference_scale(avg),'coverageAdequateForSizeSelection':adequate,'observedTaskClasses':classes,'observedFamilies':families,'observedShapes':shapes,
                  'basis':'Actual solver calls including failures; endpoint rejects separate. Size is provisional until all task classes and both families/shapes are covered.'}
        write(RESULTS/a.run/'resource_estimate.json',estimate); print(estimate)

if __name__=='__main__': main()
