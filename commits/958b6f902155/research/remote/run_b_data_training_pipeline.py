"""Local R2025b B data collection, then verified transfer to remote CUDA training."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_name")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.run_name):
        raise ValueError("Invalid run name")
    root = Path(__file__).resolve().parents[2]
    output = root / "results" / "abfd" / "compute" / args.run_name
    now = lambda: datetime.now(timezone.utc).isoformat()
    config = {"kind": "B-data-then-training", "seed": 20260905, "tasks": 60,
              "controllerMode": "fullstate-tvlqr", "solverMaxIterations": 2000,
              "solverMaxCpuTime": 600, "trainingEpochs": 300}
    config["sourceRevision"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    config["sourceDirty"] = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip())
    if output.exists():
        if not args.resume or json.loads((output / "pipeline_config.json").read_text()) != config:
            raise ValueError("Existing run requires --resume and identical configuration")
    else:
        if args.resume:
            raise ValueError("Cannot resume a missing run")
        output.mkdir(parents=True)
        (output / "pipeline_config.json").write_text(json.dumps(config, indent=2))
    attempt = output / "attempts" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f")
    attempt.mkdir(parents=True)
    status = {"state": "starting", "started": now(), "supervisorPid": os.getpid(), "attempt": str(attempt)}

    def publish():
        tmp = output / "pipeline_status.json.tmp"
        tmp.write_text(json.dumps(status, indent=2), encoding="utf-8")
        tmp.replace(output / "pipeline_status.json")

    lock = None
    try:
        lock = (output / "pipeline.lock").open("x")
        lock.write(str(os.getpid())); lock.flush()
        m_quote = lambda value: str(value).replace("\\", "/").replace("'", "''")
        script = (f"cd('{m_quote(root)}'); addpath('research'); root=setupAbfdResearch();\n"
                  "tasks=makeAbfdTaskManifest('residual',60,20260905);\n"
                  f"writeAbfdJson('{m_quote(output / 'B_tasks_frozen.json')}',tasks);\n"
                  "c=makeAbfdConfig(struct('controllerMode','fullstate-tvlqr',"
                  "'solverMaxIterations',2000,'solverMaxCpuTime',600,"
                  f"'outputDir','{m_quote(output / 'B0')}'));\n"
                  "run_B0_collect_dataset(tasks,c);\n")
        stage_file = attempt / "run_B_data.m"
        stage_file.write_text(script, encoding="ascii")
        with (attempt / "matlab.stdout.log").open("xb") as stdout, (attempt / "matlab.stderr.log").open("xb") as stderr:
            process = subprocess.Popen(["D:/MATLAB/R2025b/bin/matlab.exe", "-batch", f"run('{m_quote(stage_file)}')"],
                                       cwd=root, stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr,
                                       creationflags=subprocess.CREATE_NO_WINDOW)
            status.update(state="collecting", workerPid=process.pid); publish()
            status["matlabExitCode"] = process.wait()
        if status["matlabExitCode"] != 0:
            raise RuntimeError("B data generation or physical identification failed; training was not dispatched")
        dataset = output / "B0" / "residual_dataset.json"
        if not dataset.is_file():
            raise RuntimeError("No verified residual dataset was exported")
        status["state"] = "dispatching-training"; publish()
        command = ["powershell.exe", "-NoProfile", "-NonInteractive", "-File",
                   str(root / "research/remote/Start-AbfdTraining.ps1"), "-Kind", "B",
                   "-DatasetPath", str(dataset), "-RunName", args.run_name + "-B", "-Epochs", "300"]
        with (attempt / "training_dispatch.stdout.log").open("xb") as stdout, (attempt / "training_dispatch.stderr.log").open("xb") as stderr:
            dispatched = subprocess.run(command, cwd=root, stdin=subprocess.DEVNULL, stdout=stdout,
                                        stderr=stderr, creationflags=subprocess.CREATE_NO_WINDOW, check=False)
        if dispatched.returncode != 0:
            raise RuntimeError("Dataset completed but remote dispatch failed; see training_dispatch logs")
        status["state"] = "training-dispatched"
        status["remoteRun"] = "D:/XieZhenyu/stewart-simscape-abfd/results/abfd/training/" + args.run_name + "-B"
    except Exception as error:
        status.update(state="failed", message=str(error))
    finally:
        status["finished"] = now(); publish()
        if lock is not None:
            lock.close(); (output / "pipeline.lock").unlink()
    return 0 if status["state"] == "training-dispatched" else 1


if __name__ == "__main__":
    sys.exit(main())
