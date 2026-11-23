import html
import json
import re
import subprocess
from pathlib import Path

root = Path(r"D:\NPU\Matlab_code\stewart-simscape-diffusion\docs\figs")
src = (root / "regression_vs_diffusion_wide.html").read_text(encoding="utf-8")
script = r'''<script>
const nodes=[...document.querySelectorAll('svg text')];
const a=nodes.map((n,i)=>{const b=n.getBBox();return{i,t:n.textContent,x:b.x,y:b.y,w:b.width,h:b.height}});
const hits=[];
for(let i=0;i<a.length;i++)for(let j=i+1;j<a.length;j++){
 const p=a[i],q=a[j],w=Math.min(p.x+p.w,q.x+q.w)-Math.max(p.x,q.x),h=Math.min(p.y+p.h,q.y+q.h)-Math.max(p.y,q.y);
 if(w>2&&h>2&&w*h>24)hits.push({a:p.t,b:q.t,overlap:[Math.round(w),Math.round(h)]});
}
const clipped=a.filter(p=>p.x<-1||p.y<-1||p.x+p.w>2401||p.y+p.h>801).map(p=>p.t);
let pre=document.createElement('pre');pre.id='qa';pre.textContent=JSON.stringify({count:a.length,hits,clipped});document.body.appendChild(pre);
</script>'''
qa = root / "_regression_vs_diffusion_qa.html"
qa.write_text(src.replace("</body>", script + "</body>"), encoding="utf-8")
try:
    chrome = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
    out = subprocess.run([chrome, "--headless", "--disable-gpu", "--virtual-time-budget=1000", "--dump-dom", qa.as_uri()],
                         capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=30)
    m = re.search(r'<pre id="qa">(.*?)</pre>', out.stdout, re.S)
    if not m:
        raise RuntimeError((out.stdout + out.stderr)[-2000:])
    report = json.loads(html.unescape(m.group(1)))
    print(json.dumps(report, ensure_ascii=False, indent=2))
finally:
    qa.unlink(missing_ok=True)
