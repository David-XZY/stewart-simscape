"""Capture compiler diagnostics; canonicalize private generated symbol names."""
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'artifacts' / 'native'
OUT.mkdir(parents=True, exist_ok=True)
arguments = sys.argv[1:]
mixed = '--ch5-mixed-opt' in arguments
arguments = [a for a in arguments if a != '--ch5-mixed-opt']
source = next((Path(a) for a in arguments if a.endswith('.c')), None)
object_path = Path(arguments[arguments.index('-o') + 1])
source_bytes = source.read_bytes() if source else b''
transformed = None
function_modes = []
code = source_bytes.decode('utf-8') if source else ''
# Only private static symbol prefixes differ between otherwise identical
# CasADi generations. Public exported function names are preserved.
code = re.sub(r'(#define CASADI_PREFIX\(ID\) )\w+( ## ID)', r'\1ch5_native_\2', code)
if mixed and source:
    matches = list(re.finditer(r'^static int casadi_f\d+\([^\n]*\) \{', code, re.MULTILINE))
    parts = []; last = 0
    for match in matches:
        # CasADi's generated function closing braces start at column zero.
        end = code.index('\n}', match.end()) + 2
        size = end - match.start()
        level = 'O2' if size <= 128000 else 'O0'
        parts.extend([code[last:match.start()], f'\n#pragma GCC push_options\n#pragma GCC optimize ("{level}")\n',
                      code[match.start():end], '\n#pragma GCC pop_options\n'])
        function_modes.append({'name': match.group().split('(')[0], 'bytes': size, 'level': level})
        last = end
    parts.append(code[last:]);code = ''.join(parts)
if source:
    transformed = source.with_name(source.stem + '_ch5_compiled.c')
    transformed.write_text(code, 'utf-8')
    arguments[arguments.index(str(source))] = str(transformed)
cache_arguments = [a for i, a in enumerate(arguments) if a != '-o' and (i == 0 or arguments[i-1] != '-o') and not a.endswith('.c')]
key = hashlib.sha256(code.encode('utf-8') + json.dumps({'flags': cache_arguments, 'mixed': mixed, 'bridge': 3}).encode()).hexdigest() if source else None
cached = OUT / f'{key}.obj' if key else None
if cached and cached.exists():
    shutil.copyfile(cached, object_path)
    (OUT / f'{time.time_ns()}.json').write_text(json.dumps({'cache_hit': True, 'key': key,
        'source_sha256': hashlib.sha256(source_bytes).hexdigest(), 'compiled_source_sha256': hashlib.sha256(code.encode()).hexdigest()}), 'utf-8')
    if transformed:transformed.unlink(missing_ok=True)
    sys.exit(0)
environment = dict(os.environ)
environment['PATH'] = 'C:\\msys64\\ucrt64\\bin;' + environment['PATH']
start = time.monotonic()
result = subprocess.run(['C:/msys64/ucrt64/bin/gcc.exe', *arguments], env=environment,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        creationflags=subprocess.CREATE_NO_WINDOW)
log = {'arguments': arguments, 'source_bytes': source.stat().st_size if source else 0,
       'mixed_function_modes': function_modes, 'source_sha256': hashlib.sha256(source_bytes).hexdigest(),
       'compiled_source_sha256': hashlib.sha256(code.encode()).hexdigest(), 'key': key,
       'seconds': time.monotonic() - start, 'returncode': result.returncode,
       'output': result.stdout.decode('utf-8', errors='replace')}
(OUT / f'{time.time_ns()}.json').write_text(json.dumps(log, indent=2), 'utf-8')
if result.returncode == 0 and cached:
    temporary = cached.with_suffix(f'.{os.getpid()}.pending')
    shutil.copyfile(object_path, temporary);os.replace(temporary, cached)
if transformed:
    transformed.unlink(missing_ok=True)
if result.returncode:
    print(log['output'], flush=True)
sys.exit(result.returncode)
