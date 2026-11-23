from pathlib import Path
import hashlib,json
p=Path(__file__).resolve().parent
m=json.loads((p/'MANIFEST.json').read_text(encoding='utf-8'))
for r in m['files']:
 f=p/r['path'];assert f.is_file(),r['path'];assert f.stat().st_size==r['bytes'],r['path'];assert hashlib.sha256(f.read_bytes()).hexdigest()==r['sha256'],r['path']
print('VERIFIED',len(m['files']),'files')
