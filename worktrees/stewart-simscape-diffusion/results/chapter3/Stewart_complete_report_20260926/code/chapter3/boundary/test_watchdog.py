"""Bounded real-process deadline audit; not a formal performance experiment."""
from run_boundary import RESULTS,write,read,benchmark_with_watchdog


def main():
    folder=RESULTS/'boundary_watchdog_v1'; folder.mkdir(parents=True,exist_ok=True)
    jobs=[]
    for i,budget in enumerate([.05,1]):
        jobs.append(dict(runId=f'watchdog_{i}',taskId=f'watchdog_{i}',groupId='watchdog_only',baseSceneId='dual_fixture_01',
                         split='development',parameters=[0,0,0,7.5],method='M0',B=budget,repeat=0,
                         samplingSeed=20260919,returnPolicy='physical'))
    protocol=dict(jobs=jobs,models=[],controlCount=16); entry=folder/'jobs.json'; write(entry,protocol)
    episodes=benchmark_with_watchdog(entry,'boundary_watchdog_v1')
    records=[read(folder/'calls'/j['runId']/'result.json') for j in jobs]
    assert all(not r['passed'] and not r['returnedWithinBudget'] for r in records)
    write(folder/'test_result.json',dict(passed=True,records=records,episodes=episodes,scope='Real process deadline audit only'))
    print('BD_WATCHDOG_TEST_PASSED')


if __name__=='__main__': main()
