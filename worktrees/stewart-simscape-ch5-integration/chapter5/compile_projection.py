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
source_index = next((i for i, a in enumerate(arguments) if a.endswith('.c')), None)
source = Path(arguments[source_index]) if source_index is not None else None
object_path = Path(arguments[arguments.index('-o') + 1])
source_bytes = source.read_bytes() if source else b''
transformed = None
function_modes = []
configuration = json.loads((ROOT / 'config.json').read_text('utf-8'))
optimization_bytes = configuration['native_function_optimization_bytes']
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
        # Even medium nonideal assembly functions reached 6.11 GiB at O1.
        # Bound compiler work at the already registered kernel-size limit;
        # retain O2 for small arithmetic kernels and standard float semantics.
        level = 'O0' if size > optimization_bytes else 'O2'
        body = re.sub(r'^static int (casadi_f\d+)\(',
                      r'static int __attribute__((noinline,noclone)) \1(', code[match.start():end])
        parts.extend([code[last:match.start()], f'\n#pragma GCC push_options\n#pragma GCC optimize ("{level}")\n',
                      body, '\n#pragma GCC pop_options\n'])
        function_modes.append({'name': match.group().split('(')[0], 'bytes': size, 'level': level})
        last = end
    parts.append(code[last:]);code = ''.join(parts)
if source:
    transformed = source.with_name(source.stem + '_ch5_compiled.c')
    transformed.write_text(code, 'utf-8')
    arguments[source_index] = str(transformed)
cache_arguments = [a for i, a in enumerate(arguments) if a != '-o' and (i == 0 or arguments[i-1] != '-o') and not a.endswith('.c')]
key = hashlib.sha256(code.encode('utf-8') + json.dumps({'flags': cache_arguments, 'mixed': mixed, 'bridge': 8}).encode()).hexdigest() if source else None
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
log = {'arguments': arguments, 'source_bytes': source.stat().st_size if source else 0,
       'large_assembly_threshold_bytes': optimization_bytes,
       'mixed_function_modes': function_modes, 'source_sha256': hashlib.sha256(source_bytes).hexdigest(),
       'compiled_source_sha256': hashlib.sha256(code.encode()).hexdigest(), 'key': key,
       'state': 'starting', 'seconds': 0, 'returncode': None}
stamp=time.time_ns();log_path=OUT / f'{stamp}.json';output_path=OUT / f'{stamp}.compiler.log'
log['compiler_output_file']=str(output_path);log_path.write_text(json.dumps(log, indent=2),'utf-8')
try:
    from child_job import attach_current_process
    attach_current_process();log['children_bound_to_wrapper_lifetime']=True
    # Preserve diagnostics even if the wrapper or compiler stops abruptly;
    # never accumulate unbounded compiler stdout in the Python process.
    with output_path.open('wb') as handle:
        process=subprocess.Popen(['C:/msys64/ucrt64/bin/gcc.exe', *arguments], env=environment,
            stdout=handle, stderr=subprocess.STDOUT, creationflags=subprocess.CREATE_NO_WINDOW)
        log['state']='running';log['compiler_pid']=process.pid
        log_path.write_text(json.dumps(log, indent=2),'utf-8');returncode=process.wait()
    with output_path.open('rb') as handle:output=handle.read(1024*1024).decode('utf-8',errors='replace')
    log.update(state='finished',seconds=time.monotonic()-start,returncode=returncode,output=output)
    log_path.write_text(json.dumps(log,indent=2),'utf-8')
except BaseException as error:
    log.update(state='wrapper_exception',seconds=time.monotonic()-start,error=repr(error))
    log_path.write_text(json.dumps(log,indent=2),'utf-8');raise
if returncode == 0 and cached:
    temporary = cached.with_suffix(f'.{os.getpid()}.pending')
    shutil.copyfile(object_path, temporary);os.replace(temporary, cached)
if transformed:
    transformed.unlink(missing_ok=True)
if returncode:
    print(log['output'], flush=True)
sys.exit(returncode)
