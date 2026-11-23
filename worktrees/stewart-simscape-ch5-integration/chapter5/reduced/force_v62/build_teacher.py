from pathlib import Path
import hashlib,json
red=Path(__file__).resolve().parent.parent
src=red/'ch5ForceV63Sfun.m';text=src.read_text('utf-8')
text=text.replace('ch5ForceV63Sfun','ch5TeacherV64Sfun')
old='ch5SafetyQp(q,qd,qr,vr,ar,nominal,s.previous,o,controllerRuntime,t)'
assert old in text
text=text.replace(old,'ch5TeacherSafetyV64(s.x,qr,vr,ar,nominal,s.previous,o,controllerRuntime,t)')
dest=red/'ch5TeacherV64Sfun.m';assert not dest.exists();dest.write_text(text,encoding='utf-8')
(red.parent/'artifacts/control_force_v62/teacher_source.json').write_text(json.dumps(dict(source=src.name,source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),output=dest.name,output_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),formal=False),indent=2),encoding='utf-8')
