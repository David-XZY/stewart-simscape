"""Run replay and complete paired V90 development runs in one live worker."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import dispatch
from dispatch_repair_v89 import wait

OUT = dispatch.OUT / 'teacher_repair_v90'


def verify():
    identity = json.loads((OUT/'identity.json').read_text(encoding='utf-8-sig'))
    for name, expected in {**identity['original_sources'], **identity['sources']}.items():
        assert hashlib.sha256((dispatch.ROOT/name).read_bytes()).hexdigest() == expected, name


if __name__ == '__main__':
    verify()
    replay = wait('v90-retreat-replay-20261015')
    assert replay['state'] == 'complete', replay
    dispatch.wait = wait
    sys.argv = [sys.argv[0], '--name', 'teacher_repair_v90_full', 'ch5SpeedTeacherRepairV90']
    dispatch.main()
    verify()
    subprocess.run([sys.executable, str(Path(__file__).with_name('compare_teacher_v90.py'))], check=True)
