"""Read-only workspace inventory; writes only this cleanup's receipt folder."""
import collections,ctypes,json,os,shutil,stat,time
from pathlib import Path

ROOT=Path('D:/NPU/Matlab_code').resolve()
OUT=ROOT/'cleanup_20261123'
get_size=ctypes.windll.kernel32.GetCompressedFileSizeW
get_size.argtypes=[ctypes.c_wchar_p,ctypes.POINTER(ctypes.c_ulong)]
get_size.restype=ctypes.c_ulong

def allocated(path):
    high=ctypes.c_ulong();low=get_size(str(path),ctypes.byref(high))
    return (int(high.value)<<32)+int(low)

def main():
    OUT.mkdir(exist_ok=True);started=time.time();top=collections.Counter();groups=collections.Counter();extensions=collections.Counter();largest=[];links=[];errors=[];count=0
    before={d:dict(zip(('total','used','free'),shutil.disk_usage(d+':/'))) for d in ('C','D')}
    todo=[ROOT]
    with (OUT/'inventory.jsonl').open('w',encoding='utf-8') as log:
        while todo:
            directory=todo.pop()
            if directory==OUT:continue
            try:entries=list(os.scandir(directory))
            except OSError as e:errors.append([str(directory),str(e)]);continue
            for entry in entries:
                try:
                    info=entry.stat(follow_symlinks=False);rel=Path(entry.path).relative_to(ROOT).as_posix()
                    if info.st_file_attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT:
                        links.append(dict(path=rel,directory=entry.is_dir(follow_symlinks=False),bytes=info.st_size));continue
                    if entry.is_dir(follow_symlinks=False):todo.append(Path(entry.path));continue
                    if not entry.is_file(follow_symlinks=False):continue
                    parts=rel.split('/');size=info.st_size
                    row=dict(path=rel,bytes=size,mtime_ns=info.st_mtime_ns,inode=info.st_ino,device=info.st_dev,attributes=info.st_file_attributes)
                    if size>=1024**2:row['allocated_bytes']=allocated(entry.path)
                    log.write(json.dumps(row,ensure_ascii=False)+'\n');count+=1
                    top[parts[0] if len(parts)>1 else '[root files]']+=size
                    groups['/'.join(parts[:min(3,len(parts)-1)]) or '[root files]']+=size
                    extensions[Path(entry.path).suffix.lower()]+=size
                    if size>=128*1024**2:largest.append(row)
                except OSError as e:errors.append([entry.path,str(e)])
    summary=dict(created=time.strftime('%Y-%m-%d %H:%M:%S'),seconds=time.time()-started,file_count=count,total_logical_bytes=sum(top.values()),disk_before=before,
                 top=top.most_common(),groups=groups.most_common(),extensions=extensions.most_common(),largest=sorted(largest,key=lambda x:-x['bytes']),reparse_points=links,errors=errors)
    (OUT/'inventory_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(count=count,logical_GiB=sum(top.values())/2**30,seconds=summary['seconds'],top=[(k,round(v/2**30,3)) for k,v in top.most_common(20)],groups=[(k,round(v/2**30,3)) for k,v in groups.most_common(35)],extensions=[(k,round(v/2**30,3)) for k,v in extensions.most_common(15)],largest=[(v['path'],round(v['bytes']/2**30,3),round(v.get('allocated_bytes',0)/2**30,3)) for v in summary['largest'][:15]],reparse_count=len(links),errors=errors),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
