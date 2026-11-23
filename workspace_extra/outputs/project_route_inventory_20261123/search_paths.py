"""Search the saved file inventory without rescanning or modifying the workspace."""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('terms',nargs='+',help='All terms must occur in the full path (case insensitive).')
p.add_argument('--limit',type=int,default=60)
p.add_argument('--kind',default=None,help='Optional category, e.g. source, archive, document_or_table.')
a=p.parse_args();terms=[x.casefold() for x in a.terms];found=0;shown=0
with (Path(__file__).parent/'files.jsonl').open(encoding='utf-8') as f:
    for line in f:
        r=json.loads(line)
        if (a.kind is None or r['kind']==a.kind) and all(x in r['path'].casefold() for x in terms):
            found+=1
            if shown<a.limit:
                print(r['path']);shown+=1
print(f'Matches: {found}; shown: {shown}. This queries the saved inventory, not current live existence.')
