"""Collect a completed campaign, render its PDF locally, and prepare visual QA.

This process does no planning or training. Strict SSH verification remains enabled.
"""
import argparse, json, subprocess, sys, time, zipfile
from pathlib import Path
from prepare import OUT,HERE,read,write,sha

REMOTE='D:/XieZhenyu/stewart-simscape-diffusion/results/chapter3/payload_pair2_nofallback_v1'
SSH=['ssh','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=8','ws']
SCP=['scp','-r','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=8']

def collect(wait):
    while True:
        try:
            command='powershell -NoProfile -Command "Get-Content -Raw -LiteralPath \''+REMOTE+'/status.json\'"'
            response=subprocess.run(SSH+[command],capture_output=True,check=True,timeout=30)
            state=json.loads(response.stdout.decode('utf-8-sig'))
            write(OUT/'remote_status.json',state)
        except (subprocess.SubprocessError,UnicodeError,json.JSONDecodeError) as error:
            write(OUT/'collection_status.json',dict(stage='connection_unavailable',error=str(error),updatedUnix=time.time()))
            if not wait:raise
            time.sleep(60);continue
        if state['stage']=='failed':
            write(OUT/'collection_status.json',dict(stage='remote_failed',remote=state));return
        if state['stage']=='four_completed':break
        write(OUT/'collection_status.json',dict(stage='waiting_for_formal_completion',remoteStage=state['stage'],updatedUnix=time.time()))
        if not wait:return
        time.sleep(60)
    # All trajectories, including failures and independent audits, are retained.
    write(OUT/'collection_status.json',dict(stage='collecting_raw_results',updatedUnix=time.time()))
    for name in ['formal','analysis_four','verification','freeze.json','preflight.json','status.json','protocol_four.json','amendment_no_ihsid.json','scope_change_stop.json']:
        subprocess.run(SCP+['ws:'+REMOTE+'/'+name,str(OUT)],check=True)
    protocol=read(OUT/'protocol_four.json');audit=read(OUT/'formal/audit.json')
    assert audit['passed'] and audit['calls']==8760
    for j in protocol['jobs']:
        folder=OUT/'formal/calls'/j['runId'];r=read(folder/'result.json')
        assert r['configHash']==protocol['configHash'] and (folder/'trajectory.mat').is_file()
    # The local renderer executes the required PDF authoring marker itself.
    subprocess.run([sys.executable,str(HERE/'report_figures4.py'),'--pdf'],check=True)
    qa=OUT/'analysis_four/pdf_qa';qa.mkdir(exist_ok=True)
    subprocess.run(['pdftoppm','-scale-to','1600','-png',str(OUT/'analysis_four/四方法无回退重测图册.pdf'),str(qa/'page')],check=True)
    pages=list(qa.glob('page-*.png'));assert len(pages)==6
    archive=OUT/'四方法无回退重测图表数据包.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for folder in [OUT/'analysis_four',HERE]:
            for f in sorted(folder.rglob('*')):
                if not f.is_file() or '__pycache__' in f.parts or 'pdf_qa' in f.parts:continue
                z.write(f,('analysis' if folder==OUT/'analysis_four' else 'code')+'/'+str(f.relative_to(folder)))
        for f in [OUT/'protocol_four.json',OUT/'amendment_no_ihsid.json',OUT/'freeze.json',OUT/'preflight.json',OUT/'formal/audit.json']:
            z.write(f,'provenance/'+f.name)
    write(OUT/'collection_status.json',dict(stage='collected_rendered_pending_visual_review',updatedUnix=time.time(),calls=8760,pdfPages=6,archiveBytes=archive.stat().st_size,archiveSha256=sha(archive)))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--wait',action='store_true');args=p.parse_args();collect(args.wait)
