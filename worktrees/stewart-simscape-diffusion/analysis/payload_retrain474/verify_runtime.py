import time
import traceback
from common import *
from runner import matlab,ml

if __name__=='__main__':
    try:
        atomic(OUT/'runtime_verification_status.json',dict(status='running'))
        matlab(f'p474Smoke({ml(SOURCE/"dataset.json")},{ml(OUT/"smoke_remote.json")});','runtime_smoke')
        matlab(f'p474Integration({ml(SOURCE/"dataset.json")},{ml(SOURCE/"library.json")},{ml(OUT/"integration.json")});','runtime_integration')
        atomic(OUT/'runtime_verification_status.json',dict(status='passed',updatedUnix=time.time()))
    except BaseException:
        atomic(OUT/'runtime_verification_status.json',dict(status='failed',error=traceback.format_exc()));raise
