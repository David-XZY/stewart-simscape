"""Run an ABFD job detached from SSH, retaining logs, lock and final exit status."""
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone


def now():
    return datetime.now(timezone.utc).isoformat()


def main():
    job_path = Path(sys.argv[1]).resolve()
    job = json.loads(job_path.read_text(encoding="utf-8-sig"))
    base = Path("D:/XieZhenyu").resolve()
    for value in (job_path, job["workingDirectory"], job["outputDirectory"], job["lockFile"]):
        if not Path(value).resolve().is_relative_to(base):
            raise ValueError("Job paths must stay under D:/XieZhenyu")
    output = Path(job["outputDirectory"])
    output.mkdir(parents=True, exist_ok=True)
    status = {"state": "starting", "started": now(), "supervisorPid": os.getpid(), "exitCode": None}

    def publish():
        temporary = output / "status.json.tmp"
        temporary.write_text(json.dumps(status, indent=2), encoding="utf-8")
        temporary.replace(output / "status.json")

    lock = None
    try:
        lock = Path(job["lockFile"]).open("x", encoding="ascii")
        lock.write(str(os.getpid()))
        lock.flush()
        env = dict(os.environ, TEMP="D:/XieZhenyu/tmp", TMP="D:/XieZhenyu/tmp", PYTHONUNBUFFERED="1")
        command = [job["executable"], *job["arguments"]]
        with (output / "stdout.log").open("xb") as stdout, (output / "stderr.log").open("xb") as stderr:
            process = subprocess.Popen(command, cwd=job["workingDirectory"], env=env,
                                       stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr,
                                       creationflags=subprocess.CREATE_NO_WINDOW)
            status.update(state="running", workerPid=process.pid)
            publish()
            status["exitCode"] = process.wait()
        if status["exitCode"] != 0:
            raise RuntimeError(f"Worker exited with code {status['exitCode']}")
        status["state"] = "complete"
    except Exception as error:
        status.update(state="failed", message=str(error))
    finally:
        status["finished"] = now()
        publish()
        if lock is not None:
            lock.close()
            Path(job["lockFile"]).unlink()
    return 0 if status["state"] == "complete" else 1


if __name__ == "__main__":
    sys.exit(main())
