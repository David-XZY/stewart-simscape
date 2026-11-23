"""Generate all frozen F tasks in independent MATLAB workers, audit, then train.

All runtime paths stay below D:/XieZhenyu. A completed candidate is never rerun
on observation timeout. Explicit resume checks live worker handles first.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import ctypes
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def now():
    return datetime.now(timezone.utc).isoformat()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sequence(value):
    return value if isinstance(value, list) else [value]


def rows(value, count):
    if count == 1 and value and not isinstance(value[0], list):
        return [value]
    if len(value) != count:
        raise ValueError("Dataset row count mismatch")
    return value


def merge_batch(output):
    """Reject missing, mixed-source, or falsely accepted shards before training."""
    output = Path(output)
    manifest = read(output / "task_manifest.json")
    fields = ("x", "y", "groups", "split", "objective", "routeMode", "startIndex",
              "candidateSeconds", "solveSeconds", "iterations", "retryCount")
    merged = {key: [] for key in fields}
    attempts_all = []
    for task in manifest["tasks"]:
        folder = output / "tasks" / task["taskId"]
        status = read(folder / "task_status.json")
        expected = {"state": "complete", "taskId": task["taskId"], "taskHash": task["taskHash"],
                    "configHash": manifest["configHash"], "implementationHash": manifest["implementationHash"]}
        if any(status.get(key) != value for key, value in expected.items()):
            raise ValueError(f"Mixed or incomplete shard: {task['taskId']}")
        attempts = sequence(read(folder / "F1" / "proposal_attempts.json"))
        if [a["start"] for a in attempts] != list(range(1, manifest["candidateCount"] + 1)):
            raise ValueError("Missing or duplicate start")
        if any(a["taskId"] != task["taskId"] or type(a["passed"]) is not bool for a in attempts):
            raise ValueError("Wrong attempt identity")
        accepted = [a["start"] for a in attempts if a["passed"]]
        if status["passed"] != len(accepted) or status["attempted"] != len(attempts):
            raise ValueError("Acceptance count mismatch")
        attempts_all.extend(attempts)
        if not accepted:
            if status["datasetHash"] or (folder / "F1" / "proposal_dataset.json").exists():
                raise ValueError("Zero-success shard contains a dataset")
            continue
        data_file = folder / "F1" / "proposal_dataset.json"
        if sha(data_file) != status["datasetHash"]:
            raise ValueError("Shard dataset hash mismatch")
        data = read(data_file)
        if data["implementationHash"] != manifest["generatorHash"]:
            raise ValueError("Generator source mismatch")
        count = len(accepted)
        if sequence(data["startIndex"]) != accepted:
            raise ValueError("Accepted rows do not match terminal attempts")
        for field in fields:
            values = rows(data[field], count) if field in ("x", "y") else sequence(data[field])
            if len(values) != count:
                raise ValueError(f"Wrong {field} row count")
            if field in ("groups", "split"):
                required = task["taskId"] if field == "groups" else task["split"]
                if any(v != required for v in values):
                    raise ValueError("Group identity or split mismatch")
            merged[field].extend(values)
    if not merged["x"] or set(merged["split"]) != {"train", "validation", "test"}:
        raise ValueError("Complete dataset needs feasible rows in every frozen split")
    merged.update(schemaVersion=1, kind="trajectory-proposal-dataset",
                  implementationHash=manifest["generatorHash"], configHash=manifest["configHash"],
                  frozenTaskCount=len(manifest["tasks"]), frozenTasks=manifest["tasks"],
                  trainingScope="All 60 frozen tasks attempted; only densely accepted starts are training rows.")
    write(output / "proposal_dataset.json", merged)
    write(output / "proposal_attempts.json", attempts_all)
    report = {"state": "complete", "tasks": len(manifest["tasks"]), "attempts": len(attempts_all),
              "accepted": len(merged["x"]), "rejected": len(attempts_all)-len(merged["x"]),
              "successfulGroups": len(set(merged["groups"])), "datasetHash": sha(output / "proposal_dataset.json"),
              "groupCounts": {s: len({g for g, v in zip(merged["groups"], merged["split"]) if v == s})
                              for s in ("train", "validation", "test")}}
    write(output / "collection_report.json", report)
    return report


def pid_alive(pid):
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.OpenProcess.restype = ctypes.c_void_p
    kernel.WaitForSingleObject.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    handle = kernel.OpenProcess(0x100000, False, int(pid))
    if not handle:
        error = ctypes.get_last_error()
        if error == 87:
            return False
        raise OSError(error, "Cannot establish whether recorded worker is alive")
    try:
        result = kernel.WaitForSingleObject(handle, 0)
        if result not in (0, 258):
            raise OSError("Cannot inspect recorded worker handle")
        return result == 258
    finally:
        kernel.CloseHandle(handle)


def assert_source(project, expected=None):
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=project, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=project, text=True).strip()
    if dirty or (expected and revision != expected):
        raise ValueError("Batch requires unchanged, clean tracked source")
    return revision


def run_process(command, project, directory):
    directory.mkdir(parents=True, exist_ok=False)
    temporary = directory / "tmp"
    temporary.mkdir()
    env = dict(os.environ, TEMP=str(temporary), TMP=str(temporary), PYTHONUNBUFFERED="1",
               OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    status = {"state": "starting", "started": now(), "supervisorPid": os.getpid(), "command": command}
    try:
        with (directory / "stdout.log").open("xb") as stdout, (directory / "stderr.log").open("xb") as stderr:
            process = subprocess.Popen(command, cwd=project, env=env, stdin=subprocess.DEVNULL,
                                       stdout=stdout, stderr=stderr, creationflags=subprocess.CREATE_NO_WINDOW)
            status.update(state="running", workerPid=process.pid)
            write(directory / "status.json", status)
            status["exitCode"] = process.wait()
        status["state"] = "complete" if status["exitCode"] == 0 else "failed"
    except Exception as error:
        status.update(state="failed", message=str(error))
    status["finished"] = now()
    write(directory / "status.json", status)
    if status["state"] != "complete":
        raise RuntimeError(f"Worker failed; see {directory}")


def task_order(tasks):
    # A frozen interleaving starts all three splits early without replacing tasks.
    by_split = {s: [t for t in tasks if t["split"] == s] for s in ("train", "validation", "test")}
    return [values[k] for k in range(max(map(len, by_split.values())))
            for values in by_split.values() if k < len(values)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    parser.add_argument("--workers", type=int, default=4, choices=range(1, 9))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--no-train", action="store_true")
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    output = Path(args.output).resolve()
    base = Path("D:/XieZhenyu").resolve()
    if not output.is_relative_to(project / "results") or not project.is_relative_to(base):
        raise ValueError("Use an output directory under the isolated remote checkout's results")
    output.mkdir(parents=True, exist_ok=True)
    revision = assert_source(project)
    source_file = output / "source_manifest.json"
    source = {"revision": revision, "workers": args.workers, "trainingEnabled": not args.no_train,
              "supervisorSha256": sha(__file__), "mathThreadsPerWorker": 1}
    if source_file.exists():
        if not args.resume or read(source_file) != source:
            raise ValueError("Existing batch needs explicit compatible --resume")
        for file in output.glob("jobs/*/status.json"):
            old = read(file)
            if old.get("state") == "running" and pid_alive(old["workerPid"]):
                raise RuntimeError("A recorded MATLAB/training process is still alive; no restart")
    else:
        if args.resume:
            raise ValueError("Cannot resume a missing batch")
        write(source_file, source)
    lock_file = output / "pipeline.lock"
    # A stale lock requires explicit inspection/removal, never automatic deletion.
    lock = lock_file.open("x", encoding="ascii")
    lock.write(str(os.getpid()))
    lock.flush()
    state = {"state": "starting", "started": now(), "supervisorPid": os.getpid(), "completedTasks": []}
    def publish():
        write(output / "pipeline_status.json", state)
    def matlab(stage, index=0):
        assert_source(project, revision)
        job = output / "jobs" / f"{stage}_{index:02d}_{datetime.now(timezone.utc):%Y%m%dT%H%M%S%f}"
        script = output / f"run_{stage}_{index:02d}.m"
        quote = lambda value: str(value).replace("\\", "/").replace("'", "''")
        script.write_text(f"addpath('{quote(project / 'research')}');\n"
                          f"addpath('{quote(project / 'research' / 'remote')}');\n"
                          f"run_abfd_f_batch_stage('{stage}','{quote(output)}',{index});\n", encoding="ascii")
        run_process(["D:/matlab2019a/bin/matlab.exe", "-singleCompThread", "-batch", f"run('{quote(script)}')"], project, job)
    try:
        publish()
        if not (output / "task_manifest.json").exists():
            matlab("prepare")
        manifest = read(output / "task_manifest.json")
        if not manifest["tasks"] or len(manifest["tasks"]) != 60:
            raise ValueError("Full run must contain all 60 frozen tasks")
        order = task_order(manifest["tasks"])
        write(output / "queue.json", [t["index"] for t in order])
        state["state"] = "collecting"
        publish()
        def task_job(task):
            status_file = output / "tasks" / task["taskId"] / "task_status.json"
            if not status_file.exists():
                matlab("task", task["index"])
            return task["taskId"]
        errors = []
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(task_job, task): task for task in order}
            for future in as_completed(futures):
                try:
                    state["completedTasks"].append(future.result())
                except Exception as error:
                    errors.append({"taskId": futures[future]["taskId"], "error": str(error)})
                state["workerFailures"] = errors
                publish()
        if errors:
            raise RuntimeError("Some task workers failed; inspect logs before explicit resume")
        state["collection"] = merge_batch(output)
        state["state"] = "auditing"
        publish()
        matlab("audit")
        audit = read(output / "multimodality_audit.json")
        if not args.no_train:
            state["state"] = "training"
            publish()
            for mode in ["nearest", "mlp"] + (["diffusion"] if audit["diffusionApproved"] else []):
                assert_source(project, revision)
                model = output / "models" / mode
                asset_file = model / "proposal_model.json"
                if asset_file.exists():
                    asset = read(asset_file)
                    if asset.get("status") != "trained" or asset.get("mode") != mode or asset.get("datasetHash") != sha(output / "proposal_dataset.json"):
                        raise RuntimeError("Existing trained asset does not match this batch")
                    continue
                command = [sys.executable, str(project / "research/f_trajectory_proposal/train_trajectory_proposal.py"),
                           str(output / "proposal_dataset.json"), str(model), "--mode", mode, "--device", "cuda",
                           "--epochs", "300"]
                if mode == "diffusion":
                    command += ["--audit", str(output / "multimodality_audit.json")]
                if model.exists() and any(model.iterdir()):
                    if not (model / "checkpoint.pt").is_file():
                        raise RuntimeError("Nonempty training output has no resumable checkpoint")
                    command += ["--resume"]
                run_process(command, project, output / "jobs" / f"train_{mode}_{datetime.now(timezone.utc):%Y%m%dT%H%M%S%f}")
            state["diffusionState"] = "trained" if audit["diffusionApproved"] else "not-authorized-by-multimodality-audit"
        state["state"] = "complete"
    except Exception as error:
        state.update(state="failed", message=str(error))
    finally:
        state["finished"] = now()
        publish()
        lock.close()
        lock_file.unlink()
    return 0 if state["state"] == "complete" else 1


if __name__ == "__main__":
    sys.exit(main())
