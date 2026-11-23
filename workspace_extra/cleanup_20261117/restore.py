"""Restore archived source/results and verified duplicates to their original paths."""
import argparse
from collections import defaultdict
import hashlib
from pathlib import Path
import shutil
import tempfile
import zipfile

from cleanup import ARCHIVE, RECORD, ROOT, command, contained, load_manifest, read_json, sha


def select(prefix):
    key = prefix.replace('\\','/').rstrip('/').casefold()
    return [row for row in load_manifest()
            if row['action'] in ('archive','duplicate')
            and (row['path'].casefold() == key or row['path'].casefold().startswith(key+'/'))]


def restore(rows):
    archives = defaultdict(list)
    for row in rows:
        dest = contained(ROOT, row['path'])
        if dest.exists():
            if not dest.is_file() or sha(dest) != row['sha256']:
                raise ValueError(f'Existing file differs: {dest}')
            continue
        if row['action'] == 'archive':
            archives[row['archive']].append(row)
        else:
            archives[None].append(row)

    for name, group in archives.items():
        with tempfile.TemporaryDirectory(prefix='restore_', dir=RECORD) as temp:
            scratch = contained(RECORD, Path(temp).name)
            if name:
                archive = contained(ARCHIVE, name)
                receipt = read_json(RECORD/'verified'/f'{name}.json')
                if sha(archive) != receipt['sha256']:
                    raise ValueError(f'Archive checksum differs: {archive}')
                wanted = scratch/'requested.txt'
                wanted.write_text('\n'.join(r['path'].replace('/','\\') for r in group)+'\n',encoding='utf-8')
                payload = scratch/'payload'
                command(['x','-bb0','-y','-spd','-scsUTF-8',f'-o{payload}',archive,'@'+str(wanted)],
                        ROOT,RECORD/'restore.log')
                for row in group:
                    source = contained(payload, row['path'])
                    if sha(source) != row['sha256']:
                        raise ValueError(f'Wrong file checksum: {row["path"]}')
                for row in group:
                    install(contained(payload, row['path']), row)
            else:
                for row in group:
                    backing = row['retained']
                    with zipfile.ZipFile(contained(ROOT,backing['path'])) as archive:
                        temp_path = scratch/'zip_member'
                        with archive.open(backing['member']) as source, temp_path.open('xb') as output:
                            shutil.copyfileobj(source,output)
                        if sha(temp_path) != row['sha256']:
                            raise ValueError(f'Wrong ZIP member checksum: {row["path"]}')
                        install(temp_path,row)


def install(source, row):
    dest = contained(ROOT, row['path'])
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest = contained(ROOT,row['path'])
    source.rename(dest)  # Windows fails if the destination appeared during extraction.
    print('Restored', row['path'])


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--prefix', required=True, help='Original workspace-relative file or directory')
    cli.add_argument('--apply', action='store_true', help='Without this flag, list selected items only.')
    args = cli.parse_args()
    rows = select(args.prefix)
    if not rows:
        cli.error('No archived or duplicate files match the prefix.')
    print(f'{len(rows):,} files, {sum(r["bytes"] for r in rows)/1024**3:.3f} GiB')
    if args.apply:
        restore(rows)
    else:
        for row in rows[:25]:
            print(row['path'], '->', row.get('archive') or row['retained']['path'])
        if len(rows) > 25:
            print(f'... {len(rows)-25:,} more (consult manifest.jsonl).')
