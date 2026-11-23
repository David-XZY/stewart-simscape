import copy
import traceback
from common import *
from runner import matlab,ml

def main():
    d=read(SOURCE/'dataset.json');v=read(OUT/'validation.json')
    invalid=next(r for r in v['tasks'] if r['task']['endpointReason']=='endpoint_leg')
    positive=next(r for r in d['rows'] if r['split']=='train' and r['task']['family']=='open')
    good=spec_from_task(positive['task']);good['taskId']='verification_static_feasible';good['qGoal']=copy.deepcopy(good['q0']);good['parameters'][2]=good['q0'][5]
    folder=OUT/'diagnostic_verification';folder.mkdir(exist_ok=True)
    cases=[dict(spec=spec_from_task(invalid['task']),statePaths=[]),dict(spec=good,statePaths=[])]
    path=OUT/'diagnostic_verification_request.json';atomic(path,dict(output=str(folder),cases=cases))
    matlab(f'p474Diagnose({ml(path)});','diagnostic_verification')
    bad=read(folder/(invalid['taskId']+'.json'));ok=read(folder/(good['taskId']+'.json'))
    assert bad['exclude'] and bad['classification']=='necessary_endpoint_constraint_violated'
    assert not ok['exclude'] and ok['classification']=='feasible_common_algorithm_failure'
    assert len(ok['routes'])==2 and all(r['found'] for r in ok['routes'])
    atomic(OUT/'diagnostic_tests.json',dict(passed=True,endpointExclusion=True,ihsid120Witness=True,spline24Witness=True,scope='synthetic_static_case_verification_not_benchmark'))

if __name__=='__main__':
    try:main()
    except BaseException:
        atomic(OUT/'diagnostic_tests.json',dict(passed=False,error=traceback.format_exc()));raise
