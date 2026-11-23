"""Validate the scheduling-only correction after preserving V90/V91 failures."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import dispatch
from dispatch_repair_v89 import wait


def verify():
    out=dispatch.OUT/'teacher_repair_v92';identity=json.loads((out/'identity.json').read_text(encoding='utf-8-sig'))
    for name,sha in {**identity['original_sources'],**identity['sources']}.items():
        assert hashlib.sha256((dispatch.ROOT/name).read_bytes()).hexdigest()==sha,name


if __name__=='__main__':
    verify();state=dispatch.state()
    assert state['state']=='complete' and state['id']=='teacher_repair_v91_selective-2-ch5SpeedTeacherSelectiveV91',state
    dispatch.wait=wait
    sys.argv=[sys.argv[0],'--name','teacher_repair_v92_final','ch5SpeedTeacherBudgetAuditV92','ch5SpeedTeacherSelectiveV92']
    dispatch.main();verify()
    subprocess.run([sys.executable,str(Path(__file__).with_name('report_teacher_v92.py'))],check=True)
