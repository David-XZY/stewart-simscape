"""Summarize the frozen direct / bounded-repair / IHSID diagnostic."""
import argparse
import csv
import json
import statistics
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    read = lambda p: json.loads(p.read_text(encoding='utf-8-sig'))
    tasks = [t for t in read(args.source/'manifest.json')['tasks'] if t['split'] == 'test']
    rows = [read(p) for p in sorted(args.run.glob('*/summary.json'))]
    assert len(rows) == 20 and {r['taskId'] for r in rows} == {t['taskId'] for t in tasks}
    for r in rows:
        assert r['finalPassed'] == (r['acceptedStage'] != 'failed')
        assert r['directTaskPassed'] == (r['directCandidatePasses'] > 0)
        assert r['needsIhsid'] == (not (r['directTaskPassed'] or r['repairTaskPassed']))
        assert r['repairAttempted'] == (not r['directTaskPassed'])

    def timings(group, key):
        values = [r[key] for r in group]
        return {'count': len(values), 'median': statistics.median(values) if values else None,
                'mean': statistics.mean(values) if values else None,
                'min': min(values) if values else None, 'max': max(values) if values else None}

    def summarize(group):
        repairs = [r for r in group if r['repairAttempted']]
        repaired = [r for r in group if r['repairTaskPassed']]
        direct = sum(r['directTaskPassed'] for r in group)
        needs = sum(r['needsIhsid'] for r in group)
        return {'tasks': len(group), 'candidateCount': len(group)*8,
                'directCandidatePasses': sum(r['directCandidatePasses'] for r in group),
                'directTaskPasses': direct, 'repairEligibleTasks': len(repairs),
                'repairTaskPasses': len(repaired), 'repairConditionalPassRate': len(repaired)/len(repairs) if repairs else None,
                'withoutIhsidPasses': direct+len(repaired), 'needsIhsidTasks': needs,
                'needsIhsidRate': needs/len(group), 'finalTaskPasses': sum(r['finalPassed'] for r in group),
                'repairSecondsAllEligible': timings(repairs, 'repairSeconds'),
                'repairSecondsSuccessful': timings(repaired, 'repairSeconds'),
                'directAcceptedOnlineSeconds': timings([r for r in group if r['directTaskPassed']], 'onlineSeconds'),
                'repairedAcceptedOnlineSeconds': timings(repaired, 'onlineSeconds'),
                'directCheckSeconds': timings(group, 'directSeconds'),
                'onlineSeconds': timings(group, 'onlineSeconds'),
                'baselineOnlineSeconds': timings(group, 'oldOnlineSeconds'),
                'fasterTasks': sum(r['onlineSeconds'] < r['oldOnlineSeconds'] for r in group)}

    summary = {'run': args.run.name, 'source': args.source.name,
               'protocol': 'Same unguided weights/noise/candidates/ranking. Existing dpQuickFit parameters frozen before test; 40 iterations, fixed endpoints and duration, unchanged dense acceptance.',
               'timingScope': 'Fresh sampling, ranking, direct checks, bounded repair and any real IHSID fallback included. Supplemental census of remaining original candidates excluded. Two MATLAB R2019a workers on ws, singleCompThread as in source benchmark.',
               'limitation': 'Retrospective diagnostic on the same 20 held-out tasks; no new independent generalization claim. Do not tune repair parameters against these results.',
               'overall': summarize(rows),
               'byScene': {f: summarize([r for r in rows if r['family'] == f]) for f in ('simple', 'hood')}}
    with (args.run/'measurements.csv').open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (args.run/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(summary, allow_nan=False))


if __name__ == '__main__':
    main()
