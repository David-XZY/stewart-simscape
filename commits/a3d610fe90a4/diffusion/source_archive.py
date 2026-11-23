"""Package only planner source and dependencies, excluding private/runtime files."""
import sys
import zipfile
from pathlib import Path
root, output = map(Path, sys.argv[1:])
output.parent.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
    for folder in ('diffusion', 'opt_minimal/core', 'opt_minimal/ihsid', 'src'):
        for file in (root/folder).rglob('*'):
            if file.is_file() and file.suffix in {'.m', '.py', '.ps1', '.md'}:
                archive.write(file, file.relative_to(root))
print(f'Packaged planner source: {output.stat().st_size} bytes')
