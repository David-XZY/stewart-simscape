"""Durably retain transient terminal job states during the sequential handoff."""
from pathlib import Path
import json
import time
from datetime import datetime,timezone
import dispatch

out=dispatch.OUT/'teacher_repair_v91/status_history';out.mkdir(exist_ok=True)
last=None;deadline=time.monotonic()+7200
while time.monotonic()<deadline:
    value=dispatch.state()
    key=(value.get('id'),value.get('state'),value.get('pid'))
    if key!=last and value:
        event=dict(observed_utc=datetime.now(timezone.utc).isoformat(),**value)
        with (out/'events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(event)+'\n')
        if value.get('state') in ['complete','failed']:
            name=value['id'];assert all(c.isalnum() or c in '-_' for c in name)
            path=out/(name+'.json')
            if not path.exists():path.write_text(json.dumps(event,indent=2),encoding='utf-8')
        last=key
    if value.get('id')=='teacher_repair_v91_selective-2-ch5SpeedTeacherSelectiveV91' and value.get('state') in ['complete','failed']:
        print('Final V91 worker state archived:',value.get('state'),flush=True);break
    time.sleep(.1)
else:raise TimeoutError('Worker state archive timeout')
