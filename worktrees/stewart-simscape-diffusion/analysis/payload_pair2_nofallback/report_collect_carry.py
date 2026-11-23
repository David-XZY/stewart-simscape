"""Collect real calls and separately labeled inherited budget evaluations."""
import argparse,json,subprocess,sys,time,zipfile
from prepare import ROOT,HERE,OUT,read,write,sha
from report_collect import SSH,SCP,REMOTE

def collect(wait):
    while True:
        try:
            command='powershell -NoProfile -Command "Get-Content -Raw -LiteralPath \''+REMOTE+'/status.json\'"'
            response=subprocess.run(SSH+[command],capture_output=True,check=True,timeout=30)
            state=json.loads(response.stdout.decode('utf-8-sig'));write(OUT/'remote_status.json',state)
        except (subprocess.SubprocessError,UnicodeError,json.JSONDecodeError) as e:
            write(OUT/'collection_status.json',dict(stage='connection_unavailable',error=str(e)))
            if not wait:raise
            time.sleep(60);continue
        if state['stage']=='failed':
            try:
                command='powershell -NoProfile -Command "Get-Content -Raw -LiteralPath \''+REMOTE+'/recovery_status.json\'"'
                response=subprocess.run(SSH+[command],capture_output=True,check=True,timeout=30)
                recovering=json.loads(response.stdout.decode('utf-8-sig')).get('active',False)
            except (subprocess.SubprocessError,UnicodeError,json.JSONDecodeError):recovering=False
            if not recovering:write(OUT/'collection_status.json',dict(stage='remote_failed',remote=state));return
        if state['stage']=='carry_completed':break
        write(OUT/'collection_status.json',dict(stage='waiting_for_carry_completion',remoteStage=state['stage'],updatedUnix=time.time()))
        if not wait:return
        time.sleep(60)
    write(OUT/'collection_status.json',dict(stage='collecting_carry_results',updatedUnix=time.time()))
    for name in ['formal','carry','analysis_carry','verification','freeze.json','preflight.json','status.json','protocol_carry.json','amendment_carry.json','ihsid_baseline_1_2_5_10.json','carry_scope_stop.json']:
        subprocess.run(SCP+['ws:'+REMOTE+'/'+name,str(OUT)],check=True)
    audit=read(OUT/'carry/audit.json');assert audit['passed'] and audit['logicalCells']==10950
    p=read(OUT/'protocol_carry.json')
    for j in p['jobs']:
        v=read(OUT/'carry/evaluations'/(j['runId']+'.json'))
        assert v['scopeHash']==p['scopeHash']
        assert not v['inherited'] or v['sourceRow']['finalQualified']
    subprocess.run([sys.executable,str(HERE/'report_figures_carry.py'),'--pdf'],check=True)
    qa=OUT/'analysis_carry/pdf_qa';qa.mkdir(exist_ok=True)
    subprocess.run(['pdftoppm','-scale-to','1600','-png',str(OUT/'analysis_carry/五方法预算成功沿用图册.pdf'),str(qa/'page')],check=True)
    assert len(list(qa.glob('page-*.png')))==6
    archive=OUT/'五方法预算成功沿用图表数据包.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for folder in [OUT/'analysis_carry',HERE]:
            for f in sorted(folder.rglob('*')):
                if not f.is_file() or '__pycache__' in f.parts or 'pdf_qa' in f.parts:continue
                z.write(f,('analysis' if folder==OUT/'analysis_carry' else 'code')+'/'+str(f.relative_to(folder)))
        for f in [OUT/'protocol_carry.json',OUT/'amendment_carry.json',OUT/'freeze.json',OUT/'preflight.json',OUT/'carry/audit.json']:
            z.write(f,'provenance/'+f.name)
    write(OUT/'collection_status.json',dict(stage='carry_collected_rendered_pending_visual_review',updatedUnix=time.time(),logicalBudgetCells=10950,uniqueReferencedCalls=audit['uniqueReferencedCalls'],inherited=audit['inherited'],pdfPages=6,archiveBytes=archive.stat().st_size,archiveSha256=sha(archive)))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--wait',action='store_true');args=p.parse_args();collect(args.wait)
