import pathlib,json,hashlib,gzip
p=pathlib.Path(__file__).resolve().parent
m=json.loads((p/'MANIFEST.json').read_text(encoding='utf-8'))
for name,d in m['files'].items():
 f=p/name
 assert f.stat().st_size==d['bytes'],name
 assert hashlib.sha256(f.read_bytes()).hexdigest()==d['sha256'],name
for name,n in m['rowCounts'].items():
 with gzip.open(p/'data'/name,'rt',encoding='utf-8') as f:assert sum(1 for _ in f)==n,name
print('PASS: all files and archived record counts match.')
