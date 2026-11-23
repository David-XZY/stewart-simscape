"""Summarize complete, actual benchmark records without dropping failed tasks."""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--traditional-source', required=True,
                        help='State the actual traditional candidate protocol used by this run.')
    args = parser.parse_args()

    def read(name):
        return json.loads((args.run/name).read_text(encoding='utf-8-sig'))

    manifest, rows, model = read('manifest.json'), read('benchmark.json'), read('training/model.json')
    if isinstance(rows, dict): rows = [rows]
    methods = ('traditional', 'diffusion', 'guided_diffusion')
    tasks = {t['taskId']: t for t in manifest['tasks'] if t['split'] == 'test'}
    expected = {(task, method) for task in tasks for method in methods}
    assert len(rows) == len(expected) and {(r['taskId'], r['method']) for r in rows} == expected
    assert model['status'] == 'trained' and model['manifestHash'] == manifest['hash']
    for row in rows:
        assert row['family'] == tasks[row['taskId']]['family']
        assert math.isfinite(row['seconds']) and row['seconds'] >= 0

    def summarize(method, family=None):
        group = [r for r in rows if r['method'] == method and (family is None or r['family'] == family)]
        success = [r for r in group if r['passed']]
        source = args.traditional_source if method == 'traditional' else method
        return {'method': method, 'candidate_source': source, 'scene': family or 'all',
                'tasks': len(group), 'successes': len(success), 'failures': len(group)-len(success),
                'success_rate': len(success)/len(group),
                'median_online_s_all': statistics.median(r['seconds'] for r in group),
                'mean_online_s_all': statistics.mean(r['seconds'] for r in group),
                'median_online_s_success': statistics.median(r['seconds'] for r in success) if success else None,
                'mean_sampling_s_all': statistics.mean(r['samplingSeconds'] for r in group),
                'min_accepted_clearance_mm': 1000*min(r['minClearance'] for r in success) if success else None}

    totals = [summarize(m) for m in methods]
    by_scene = [summarize(m, f) for f in ('simple', 'hood') for m in methods]
    for name, table in [('benchmark_summary.csv', totals), ('benchmark_by_scene.csv', by_scene)]:
        with (args.run/name).open('w', newline='', encoding='utf-8-sig') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(table[0]))
            writer.writeheader(); writer.writerows(table)
    audit = read('delivery_audit.json') if (args.run/'delivery_audit.json').exists() else {}
    result = {'run': args.run.name, 'kind': manifest['kind'], 'manifestHash': manifest['hash'],
              'evaluationTasks': len(tasks), 'comparisons': len(rows), 'summary': totals,
              'offlineAttemptWallSecondsSum': audit.get('offlineAttemptWallSecondsSum'),
              'acceptedLabels': audit.get('acceptedLabels'),
              'trainingWallSecondsThisInvocation': model.get('trainingWallSecondsThisInvocation'),
              'epochsCompleted': model['epochsCompleted'], 'groupCounts': model['groupCounts'],
              'bestValidationNoiseMSE': model['bestValidationNoiseMSE'],
              'scope': 'All scheduled tasks included. Summed offline attempt wall time is not parallel batch elapsed time.'}
    (args.run/'run_summary.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(result, allow_nan=False))


if __name__ == '__main__':
    main()
