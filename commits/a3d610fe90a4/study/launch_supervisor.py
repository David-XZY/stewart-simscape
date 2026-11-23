"""File-logged entry point for a detached, single-run Windows supervisor."""
import os
from pathlib import Path
import runpy
import sys
import traceback

root=Path(__file__).resolve().parents[1]
run=sys.argv[1] if len(sys.argv)>1 else 'frozen_v1'
output=root/'results'/'study'/run
output.mkdir(parents=True,exist_ok=True)
with (output/'supervisor_detached.log').open('a',encoding='utf-8',buffering=1) as stream:
    original_stdout,original_stderr=sys.stdout,sys.stderr
    try:
        sys.stdout=stream; sys.stderr=stream
        print('DETACHED_SUPERVISOR_START',os.getpid(),flush=True)
        os.chdir(root)
        sys.argv=[str(root/'study'/'run_study.py'),'--run',run]
        runpy.run_path(sys.argv[0],run_name='__main__')
    except BaseException:
        traceback.print_exc(file=stream)
        raise
    finally:
        sys.stdout=original_stdout; sys.stderr=original_stderr
