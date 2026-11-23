"""Durable launcher, including post-run rendering; no PowerShell policy changes."""
import subprocess, sys
from pathlib import Path
here=Path(__file__).resolve().parent
subprocess.run([sys.executable,str(here/'runner.py'),'--stage','all'],check=True)
subprocess.run([sys.executable,str(here/'report_figures.py')],check=True)
