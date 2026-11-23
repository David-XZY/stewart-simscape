import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from child_job import attach_current_process

attach_current_process()
child = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(180)'],
                         creationflags=subprocess.CREATE_NO_WINDOW)
Path(__file__).with_suffix('.json').write_text(json.dumps({'parent_pid': os.getpid(), 'child_pid': child.pid}), 'utf-8')
time.sleep(180)
