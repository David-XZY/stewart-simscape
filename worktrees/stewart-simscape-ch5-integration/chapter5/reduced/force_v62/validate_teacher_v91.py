"""Preserve V90 failure; gate a separate V91 batch on causal replay success."""
import hashlib
import json
import subprocess
import sys
from pathlib import Path
import dispatch
from dispatch_repair_v89 import wait


def verify():
    out=dispatch.OUT/'teacher_repair_v91'
    identity=json.loads((out/'identity.json').read_text(encoding='utf-8-sig'))
    for name,sha in {**identity['original_sources'],**identity['sources']}.items():
        assert hashlib.sha256((dispatch.ROOT/name).read_bytes()).hexdigest()==sha,name


if __name__=='__main__':
    verify()
    preceding=wait('teacher_retention_guards_v90-0-ch5SpeedTeacherRetreatGuardsV90')
    assert preceding['state']=='complete',preceding
    dispatch.wait=wait
    sys.argv=[sys.argv[0],'--name','teacher_repair_v91_selective','ch5SpeedTeacherCausalReserveV91','ch5SpeedTeacherEquivalenceV91','ch5SpeedTeacherSelectiveV91']
    dispatch.main()
    verify()
    subprocess.run([sys.executable,str(Path(__file__).with_name('compare_teacher_v90.py')),'--version','v91'],check=True)
    subprocess.run([sys.executable,str(Path(__file__).with_name('plot_teacher_v90.py')),'--version','v91'],check=True)
