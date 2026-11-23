import pathlib, wave, subprocess, json, time
import numpy as np
from transcribe_local import ROOT, WORK, boundaries, stamp

exe=r'D:\NPU\InterviewCopilot\work\asr-evaluation-20260901\runtime-avx2\llama-funasr-sensevoice.exe'
model=r'D:\NPU\InterviewCopilot\work\asr-evaluation-20260901\models\sensevoice-small-q8.gguf'
with wave.open(str(ROOT/'录音16k.wav'),'rb') as f:
    params=f.getparams(); raw=f.readframes(f.getnframes())
x=np.frombuffer(raw,dtype='<i2').astype(np.float32)/32768
segs=boundaries(x,16000,17,25)
outpath=WORK/'sensevoice.jsonl'
old={json.loads(r)['index'] for r in outpath.read_text('utf-8').splitlines()} if outpath.exists() else set()
with outpath.open('a',encoding='utf-8') as out:
    for i,(s,e) in enumerate(segs):
        if i in old: continue
        a=max(0,s-4000);b=min(len(x),e+4000)
        with wave.open(str(WORK/'sensevoice_segment.wav'),'wb') as f:
            f.setparams(params);f.writeframes(raw[a*2:b*2])
        start=time.time()
        r=subprocess.run([exe,'-m',model,'-a',str(WORK/'sensevoice_segment.wav')],capture_output=True,timeout=120)
        if r.returncode: raise RuntimeError(r.stderr.decode('utf-8',errors='replace'))
        row=dict(index=i,start=s/16000,end=e/16000,text=r.stdout.decode('utf-8').strip(),processing_ms=int((time.time()-start)*1000))
        out.write(json.dumps(row,ensure_ascii=False)+'\n');out.flush()
        print(f'{i+1}/{len(segs)} {stamp(s/16000)}',flush=True)
