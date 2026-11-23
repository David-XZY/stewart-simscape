"""Generate all frozen held-out references, preserving planning failures.

Independent CPU workers use the analytic planner only. No training, model
selection, B weights, or held-out Simscape outcomes enter reference selection.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import os
from pathlib import Path
import re

from run_f_batch_pipeline import assert_source, now, pid_alive, read, run_process, sha, write


def check_tasks(tasks):
    if len(tasks) != 12 or len({task["taskId"] for task in tasks}) != 12:
        raise ValueError("Use all twelve independently frozen holdout tasks")
    if any(task["split"] != "test" or not re.fullmatch(r"task_[0-9]+", task["taskId"]) for task in tasks):
        raise ValueError("Only frozen test groups with safe identifiers are accepted")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tasks"); parser.add_argument("tasks_sha256"); parser.add_argument("output")
    parser.add_argument("--workers", type=int, default=2, choices=(1, 2))
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    tasks_file, output = Path(args.tasks).resolve(), Path(args.output).resolve()
    if not project.is_relative_to(Path("D:/XieZhenyu").resolve()) or not output.is_relative_to(project / "results") or not tasks_file.is_relative_to(project):
        raise ValueError("Use the isolated remote project and its results directory")
    if sha(tasks_file) != args.tasks_sha256.lower():
        raise ValueError("Holdout manifest differs from the pre-training freeze")
    tasks = read(tasks_file); check_tasks(tasks)
    revision = assert_source(project)
    output.mkdir(parents=True, exist_ok=True)
    source = {"revision": revision, "tasksSha256": sha(tasks_file), "workers": args.workers,
              "supervisorSha256": sha(__file__), "processHelperSha256": sha(Path(__file__).with_name("run_f_batch_pipeline.py")),
              "policy": "First dense-feasible analytic reference from at most eight ordered starts; no held-out performance selection"}
    source_file = output / "source_manifest.json"
    if source_file.exists():
        if not args.resume or read(source_file) != source:
            raise ValueError("Existing batch requires explicit matching --resume")
        for file in output.glob("jobs/*/status.json"):
            status = read(file)
            if status.get("state") == "running" and pid_alive(status["workerPid"]):
                raise RuntimeError("Recorded MATLAB worker remains live; no restart")
    else:
        if args.resume:
            raise ValueError("Cannot resume a missing batch")
        write(source_file, source)
    lock_file = output / "pipeline.lock"
    lock = lock_file.open("x", encoding="ascii"); lock.write(str(os.getpid())); lock.flush()
    state = {"state": "running", "started": now(), "supervisorPid": os.getpid(), "expected": 12,
             "completedTasks": [], "workerFailures": [], "tasksSha256": source["tasksSha256"], "references": []}

    def task_job(index, task):
        assert_source(project, revision)
        folder = output / "tasks" / task["taskId"]
        status_file = folder / "status.json"
        status = read(status_file) if status_file.exists() else {}
        if status.get("state") != "complete":
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f")
            job = output / "jobs" / (task["taskId"]+"_"+stamp)
            script = output / (task["taskId"]+"_"+stamp+".m")
            quote = lambda value: str(value).replace("\\", "/").replace("'", "''")
            script.write_text(f"addpath('{quote(project / 'research')}');\n"
                              f"addpath('{quote(project / 'research/remote')}');\n"
                              f"run_abfd_frozen_reference_task('{quote(tasks_file)}',{index},'{quote(folder)}');\n", encoding="ascii")
            run_process(["D:/matlab2019a/bin/matlab.exe", "-wait", "-singleCompThread", "-batch", f"run('{quote(script)}')"], project, job)
            status = read(status_file)
        if status.get("state") != "complete" or status.get("tasksHash") != source["tasksSha256"] or status.get("taskId") != task["taskId"] or status.get("split") != "test":
            raise ValueError("Incomplete or foreign reference outcome")
        if status["passed"] and sha(folder / "reference.mat") != status["referenceHash"]:
            raise ValueError("Selected reference changed")
        return status

    try:
        write(output / "pipeline_status.json", state)
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(task_job, i, task): task for i, task in enumerate(tasks, 1)}
            for future in as_completed(futures):
                task = futures[future]
                try:
                    status = future.result()
                    state["references"].append(status)
                    state["completedTasks"].append(task["taskId"])
                except Exception as error:
                    state["workerFailures"].append({"taskId": task["taskId"], "message": str(error)})
                write(output / "pipeline_status.json", state)
        if state["workerFailures"]:
            raise RuntimeError("A reference worker failed; inspect its terminal status before explicit resume")
        state["state"] = "complete"
        state["feasibleTasks"] = sum(row["passed"] for row in state["references"])
        state["scope"] = "All 12 frozen holdout planning outcomes retained; feasible reference is not controller or ABFD acceptance"
    except Exception as error:
        state.update(state="failed", message=str(error))
    finally:
        state["finished"] = now(); write(output / "pipeline_status.json", state)
        lock.close(); lock_file.unlink()
    return 0 if state["state"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
