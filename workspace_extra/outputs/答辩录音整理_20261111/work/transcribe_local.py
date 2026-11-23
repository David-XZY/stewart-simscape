import argparse, json, struct, subprocess, time, wave, pathlib, hashlib
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
WORK = ROOT / 'work'
APP = pathlib.Path(r'D:\NPU\InterviewCopilot')
MODEL = APP / 'work/asr-evaluation-20260901/models'
WORKER = APP / 'outputs/release-20261108-light-overlay/runtimes/funasr-nano-worker.exe'

def stamp(t):
    t = int(t)
    return f'{t//3600:02}:{t//60%60:02}:{t%60:02}'

def read_exact(f, n):
    b = bytearray()
    while len(b) < n:
        x = f.read(n-len(b))
        if not x:
            raise RuntimeError('Worker closed pipe')
        b.extend(x)
    return bytes(b)

def response(p):
    magic,version,idx,status,nt,ne,ms=struct.unpack('<6IQ',read_exact(p.stdout,32))
    text=read_exact(p.stdout,nt).decode('utf-8')
    error=read_exact(p.stdout,ne).decode('utf-8')
    assert magic==0x31524e46 and version==1
    if status==1:
        raise RuntimeError(error)
    return text,ms

def boundaries(x, sr, lo, hi):
    hop=160
    n=len(x)//hop
    energy=np.sqrt(np.mean(x[:n*hop].reshape(n,hop)**2,axis=1))
    smooth=np.convolve(energy,np.ones(15)/15,mode='same')
    start=0
    result=[]
    while start<len(x):
        if len(x)-start <= hi*sr:
            end=len(x)
        else:
            a=int((start+lo*sr)//hop)
            b=min(int((start+hi*sr)//hop),len(smooth)-1)
            end=(a+int(np.argmin(smooth[a:b])))*hop
        result.append((start,end))
        start=end
    return result

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--pass-name',required=True)
    ap.add_argument('--model',default=str(MODEL/'qwen3-0.6b-q4km.gguf'))
    ap.add_argument('--lo',type=float,default=17)
    ap.add_argument('--hi',type=float,default=25)
    ap.add_argument('--hotwords',default='')
    ap.add_argument('--segments',default='')
    ap.add_argument('--gain',type=float,default=1.0)
    args=ap.parse_args()
    with wave.open(str(ROOT/'录音16k.wav'),'rb') as f:
        sr=f.getframerate()
        assert sr==16000 and f.getnchannels()==1
        x=np.frombuffer(f.readframes(f.getnframes()),dtype='<i2').astype(np.float32)/32768
    segments=boundaries(x,sr,args.lo,args.hi)
    if args.segments:
        segments=[(int(s*sr),int(e*sr)) for s,e in json.loads(pathlib.Path(args.segments).read_text('utf-8'))]
    data_path=WORK/(args.pass_name+'.jsonl')
    old={r['index']:r for r in [json.loads(s) for s in data_path.read_text('utf-8').splitlines()]} if data_path.exists() else {}
    (WORK/(args.pass_name+'-config.json')).write_text(json.dumps(dict(model=args.model,worker=str(WORKER),duration=len(x)/sr,segments=len(segments),hotwords=args.hotwords,lo=args.lo,hi=args.hi),ensure_ascii=False,indent=2),'utf-8')
    started=time.time()
    with (WORK/(args.pass_name+'-worker.log')).open('ab') as err, data_path.open('a',encoding='utf-8') as out:
        p=subprocess.Popen([str(WORKER),'--enc',str(MODEL/'funasr-encoder-f16.gguf'),'-m',args.model,'--worker','-n','768'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err)
        try:
            response(p)
            print(f'READY {args.pass_name} duration={len(x)/sr:.2f} segments={len(segments)}',flush=True)
            hot=args.hotwords.encode('utf-8')
            for i,(s,e) in enumerate(segments):
                if i in old: continue
                # Retain a 0.25-second overlap on both sides for boundary review.
                a=max(0,s-4000); b=min(len(x),e+4000)
                samples=np.clip(x[a:b]*args.gain,-0.99,0.99).astype('<f4').tobytes()
                p.stdin.write(struct.pack('<6I',0x31514e46,1,i+1,(b-a),len(hot),0)+hot+samples)
                p.stdin.flush()
                text,ms=response(p)
                row=dict(index=i,start=s/sr,end=e/sr,audio_start=a/sr,audio_end=b/sr,text=text,processing_ms=ms)
                out.write(json.dumps(row,ensure_ascii=False)+'\n');out.flush()
                print(f'{i+1}/{len(segments)} {stamp(s/sr)}-{stamp(e/sr)} {ms/1000:.1f}s {text}',flush=True)
        finally:
            p.stdin.close()
            p.wait(timeout=30)
    print(f'COMPLETE {args.pass_name} wall={time.time()-started:.1f}',flush=True)

if __name__=='__main__': main()
