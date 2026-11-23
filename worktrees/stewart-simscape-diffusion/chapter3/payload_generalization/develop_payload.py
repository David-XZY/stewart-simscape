"""Bounded development continuation. Stops at explicit label and error gates."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time
from pg_core import read,write
from run_payload import ROOT,RESULTS,HERE

def run(max_attempts=576,train_basic=False):
    folder=RESULTS/'payload_development_job_v1'; folder.mkdir(parents=True,exist_ok=True)
    status=folder/'status.json'
    if status.exists(): raise ValueError('Existing development job: inspect status before creating a new version')
    started=time.time(); state=dict(pid=os.getpid(),startedUnix=started,state='running',stage='reference_development',maxAttemptsIncludingPilot=max_attempts,trainBasicAfterGate=train_basic)
    write(status,state)
    entry=str(HERE/'run_payload.py'); manifest=RESULTS/'payload_tasks_v1/development_manifest.json'
    def command(stage,*args):
        state.update(stage=stage,updatedUnix=time.time()); write(status,state)
        subprocess.run([sys.executable,entry,*map(str,args)],cwd=ROOT,check=True)
    try:
        command('reference_development','references','--manifest',manifest,'--run','payload_references_v1','--max-attempts',max_attempts)
        command('representation_and_label_audit','build-dataset','--sources','payload_pilot_v1','payload_reference_executor_smoke_v1','payload_references_v1','--run','payload_dataset_v1')
        command('reference_resource_estimate','resource-estimate','--run','payload_references_v1')
        report=read(RESULTS/'payload_dataset_v1/dataset_report.json'); state['labelReport']=report
        from pg_statistics import quality_caps
        try:
            caps=quality_caps(read(RESULTS/'payload_dataset_v1/quality_reference_rows.json')); write(RESULTS/'payload_dataset_v1/quality_caps.json',caps)
        except ValueError as e:
            state['qualityCalibrationReason']=str(e)
        if not report['trainingAllowed']:
            state.update(state='stopped_at_gate',stage='insufficient_qualified_labels_or_representation_damage')
        elif not train_basic:
            state.update(state='development_completed',stage='ready_for_basic_training')
        else:
            command('strong_retrieval_validation','select-retrieval','--library',RESULTS/'payload_dataset_v1/library.json','--manifest',manifest)
            for method in ['ddpm_basic','regression']:
                for seed in [20260919,20260920,20260921]:
                    command(f'train_{method}_{seed}','train','--method',method,'--dataset',RESULTS/'payload_dataset_v1/dataset.json',
                            '--output',RESULTS/'payload_models_v1'/f'{method}_{seed}','--seed',seed)
            state.update(state='basic_training_completed',stage='full_validation_and_enhancement_selection_pending')
    except Exception as e:
        state.update(state='failed',failureType=type(e).__name__,failureReason=str(e)); raise
    finally:
        state.update(updatedUnix=time.time(),elapsedSeconds=time.time()-started); write(status,state)

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--max-attempts',type=int,default=576); p.add_argument('--train-basic',action='store_true'); a=p.parse_args(); run(a.max_attempts,a.train_basic)
