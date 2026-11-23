"""Freeze original-model protocol; no training and no new test-task selection."""
import hashlib, json, random
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'results/chapter3/payload_pair2_nofallback_v1'
OLD=ROOT/'results/chapter3/payload_heldout_complete_v2'
METHODS=['ddpm_basic','ddpm_signature','regression','regression_signature','ihsid']
BUDGETS=[1,2,5,10,20]
VERSION='payload_pair2_nofallback_v1'
def read(p): return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def digest(x): return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def write(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(x,ensure_ascii=False,indent=2),encoding='utf-8');tmp.replace(p)

def prepare():
    main=read(OLD/'payload_test_orchestration_v2/main_frozen.json')
    control=read(OLD/'payload_test_orchestration_v2/controls_frozen.json')
    exclusions=read(ROOT/'results/chapter3/payload_endpoint_pass_figures_v1_20260927/data/excluded_tasks.json')
    excluded={r['taskId'] for r in exclusions};assert len(excluded)==46
    entries={m['key']:m for p in [main,control] for m in p['models']}
    hashes={k.replace('\\','/'):v for p in [main,control] for k,v in p['fileHashes'].items()}
    models=[]
    for method in METHODS[:-1]:
        for seed in [20260919,20260920,20260921]:
            e=entries[f'{method}_{seed}'];rel=e['file'].replace('\\','/').split('/stewart-simscape-diffusion/')[1]
            models.append(dict(key=e['key'],relativeFile=rel,sha256=hashes[e['file'].replace('\\','/')]))
    templates={(j['spec']['taskId'],j['repeat']):j for j in main['jobs'] if j['method']=='ihsid' and j['spec']['taskId'] not in excluded}
    assert len(templates)==438
    jobs=[]
    for key,t in sorted(templates.items()):
        for B in BUDGETS:
            block=[]
            for method in METHODS:
                j=dict(t,B=B,method=method,modelKey='' if method=='ihsid' else f"{method}_{t['trainingSeed']}")
                j['runId']=digest([VERSION,key,B,method])[:24];block.append(j)
            random.Random(digest([VERSION,key,B])).shuffle(block);jobs.extend(block)
    assert len(jobs)==10950 and len({j['spec']['baseSceneId'] for j in jobs})==20
    source_tasks=read(OLD/'payload_formal_preparation_v1/train_validation_tasks.json')['tasks']
    specs={s['taskId']:s for s in source_tasks}
    val=read(OLD/'payload_formal_dataset_v1/dataset.json')['validationTasks']
    selected=[]
    for shape in ['cylinder','box']:
        for family in ['open','dual_fixture']:
            choices=sorted(r['taskId'] for r in val if r['task']['endpointPassed'] and r['task']['payload']['shape']==shape and r['task']['family']==family)
            assert choices;selected.append(specs[choices[0]])
    pilot=[]
    for spec in selected:
        for B in [2,20]:
            for method in METHODS:
                pilot.append(dict(runId=digest([VERSION,'pilot',spec['taskId'],B,method])[:24],spec=spec,B=B,method=method,
                    returnPolicy='physical',modelKey='' if method=='ihsid' else f'{method}_20260919',
                    trainingSeed=20260919,samplingSeed=20260919,repeat=0,preselectedRepresentative=True))
    files=[]
    for directory in ['chapter3','diffusion','study','opt_minimal/core','opt_minimal/ihsid','src']:
        files.extend((ROOT/directory).rglob('*.m'))
    # pgConfig also hashes Python and protocol files.
    files.extend((ROOT/'chapter3/payload_generalization').glob('*.py'))
    files.append(ROOT/'chapter3/payload_generalization/protocol.json')
    files.extend(HERE.glob('*.m'));files.extend(HERE.glob('*.py'))
    # Reporting-only files are not invoked by this campaign or by pgConfig's numerical identity.
    files=[p for p in files if not p.name.startswith('report_') and p.name!='sgExport.m']
    source_hashes={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(set(files))}
    protocol=dict(version=VERSION,status='prepared_not_yet_pilot_verified',methods=METHODS,budgets=BUDGETS,models=models,
       nonNumericalExcluded=['report_*.py','study/sgExport.m'],originalCodeHash=main['matlabCodeHash'],libraryHash=hashes[main['libraryFile'].replace('\\','/')],
       datasetHash=hashes[main['datasetFile'].replace('\\','/')],controlCount=16,sourceHashes=source_hashes,
       batchSize=2,candidateLimit=None,sort=False,fallback=False,diffusionRepairIterations=40,regressionContinue=True,
       denseStep=.005,fineStep=.0025,watchdogGraceSeconds=30,bootstrapReplicates=2000,bootstrapSeed=20260928,
       nominalSeconds=sum(j['B'] for j in jobs),exclusions=exclusions,pilotJobs=pilot,jobs=jobs)
    protocol['configHash']=digest(protocol)
    dest=OUT/'protocol.json'
    if dest.exists() and list(OUT.glob('*/calls/*/result.json')):
        assert read(dest)['configHash']==protocol['configHash'],'Committed calls prohibit in-place protocol changes'
    write(dest,protocol)
    print(json.dumps(dict(jobs=len(jobs),pilot=len(pilot),nominalHours=protocol['nominalSeconds']/3600,hash=protocol['configHash'])))
if __name__=='__main__': prepare()
