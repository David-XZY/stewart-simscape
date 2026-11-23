"""Check protected code, selected reference paths, designs, and cleanup receipts."""
import collections,json,shutil,time
from pathlib import Path
from inventory import ROOT,OUT,allocated
from compress import sha

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))

def main():
    baseline=read(OUT/'protected_before.json');changes=[]
    for rel,expected in baseline['files'].items():
        p=ROOT/rel
        if not p.is_file() or sha(p)!=expected:changes.append(rel)
    project=ROOT/'stewart_unified_nmpc';base=project/'results/ch4_noopt_20261121'
    references=[]
    for item in read(base/'references/uniform_bernstein_v1/registry.json')['entries']:
        r=item['selected'];p=Path(r['file']);references.append(dict(task_id=item['task_id'],present=p.is_file(),hash_match=p.is_file() and sha(p)==r['sha256']))
    designs=[]
    for record in sorted((base/'designs/new_references_v1').glob('*/*/design_record.json')):
        r=read(record);p=record.parent/'design.mat';native=Path(r['native_folder'])
        designs.append(dict(folder=str(record.parent),design_present=p.is_file(),design_bytes=p.stat().st_size if p.is_file() else None,
                            library_present=(native/'controller.dll').is_file(),library_receipt_present=(native/'compiler.json').is_file()))
    modules=read(OUT/'loaded_modules_before.json');missing=[v for v in modules if not Path(v).is_file()]
    logs={};by_type=collections.Counter();compression_saved=0;deleted_saved=0;errors=[];removed=[];compressed={}
    for log in OUT.glob('*_actions.jsonl'):
        records=[json.loads(s) for s in log.open(encoding='utf-8')];logs[log.name]=dict(collections.Counter(r['status'] for r in records))
        for r in records:
            if r['status'] in ('compressed_verified','lzx_verified'):
                compression_saved+=r.get('saved_bytes',0);by_type[Path(r['path']).suffix]+=r.get('saved_bytes',0);compressed[r['path']]=r
            if r['status'] in ('concurrent_change_detected',):errors.append(r)
    for line in (OUT/'deleted_objects.jsonl').open(encoding='utf-8'):
        r=json.loads(line)
        if r['status']=='removed_regenerable_object':deleted_saved+=r['allocated_bytes'];removed.append(r)
    recreated=[r['path'] for r in removed if (ROOT/r['path']).exists()]
    # Each compressed payload was already fully rehashed after its operation.
    # Recheck presence and size now; avoid re-reading hundreds of GB while the
    # independent research thread is active.
    unavailable=[rel for rel,r in compressed.items() if not (ROOT/rel).is_file() or (ROOT/rel).stat().st_size!=r['logical_bytes']]
    metadata_bytes=sum(allocated(p) for p in OUT.rglob('*') if p.is_file())
    result=dict(created=time.strftime('%Y-%m-%d %H:%M:%S'),cleanup_complete=(OUT/'operations_complete.json').exists(),protected_files=len(baseline['files']),protected_changes=changes,reference_count=len(references),references=references,
                design_count=len(designs),designs=designs,loaded_module_count=len(modules),missing_loaded_modules=missing,
                operation_counts=logs,compressed_unique_files=len(compressed),compressed_effective_files=sum(r['after_allocated']<r['logical_bytes'] for r in compressed.values()),compressed_unavailable=unavailable,compression_saved_bytes=compression_saved,
                deleted_files=len(removed),deleted_saved_bytes=deleted_saved,recreated_cache_files=recreated,concurrent_changes=errors,
                compression_savings_by_extension=dict(by_type),cleanup_record_allocated_bytes=metadata_bytes,
                net_saved_estimate_bytes=compression_saved+deleted_saved-metadata_bytes,disk_before=read(OUT/'inventory_summary.json')['disk_before']['D'],
                disk_after=dict(zip(('total','used','free'),shutil.disk_usage(ROOT))),
                scope='All compressed files fully SHA256-checked at operation time; current Chapter 4 selected references rehashed, untouched calibrated designs checked for presence. This is a filesystem-preservation check, not a new control experiment.')
    result['passed']=not(changes or missing or errors or unavailable) and len(references)==60 and all(r['hash_match'] for r in references) and len(designs)==360 and all(r['design_present'] and r['library_present'] and r['library_receipt_present'] for r in designs)
    (OUT/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('passed','protected_files','protected_changes','reference_count','design_count','deleted_files','compressed_unique_files','compressed_unavailable','net_saved_estimate_bytes','disk_after','operation_counts')},ensure_ascii=False,indent=2))
    if not result['passed']:raise SystemExit(1)

if __name__=='__main__':main()
