"""Run the 12 frozen F test tasks with unchanged trained assets and 8-to-2 ranking."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import os
from pathlib import Path
import sys

from run_f_batch_pipeline import assert_source, now, read, run_process, sha, write


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--workers", type=int, default=2, choices=(1, 2))
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    inputs, output = args.input.resolve(), args.output.resolve()
    if not project.is_relative_to(Path("D:/XieZhenyu")):
        raise ValueError("Use an isolated remote checkout under D:/XieZhenyu")
    if any(not path.is_relative_to(project / "results") for path in (inputs, output)):
        raise ValueError("Inputs and output must be in the frozen checkout's results")
    revision = assert_source(project)
    manifest = read(inputs / "task_manifest.json")
    tasks = [task for task in manifest["tasks"] if task["split"] == "test"]
    if len(tasks) != 12 or len({task["taskId"] for task in tasks}) != 12:
        raise ValueError("All 12 predeclared test tasks are required, including zero-success tasks")
    audit = read(inputs / "import_checks/model_audit.json")
    if not audit["passed"] or sha(inputs / "proposal_dataset.json") != audit["datasetHash"]:
        raise ValueError("Input delivery audit mismatch")
    output.mkdir(parents=True, exist_ok=False)
    write(output / "source_manifest.json", {"revision": revision, "workers": args.workers,
          "supervisorSha256": sha(__file__), "datasetHash": audit["datasetHash"],
          "inputManifestHash": sha(inputs / "task_manifest.json"), "tasks": tasks,
          "selectionPolicy": "ranked-top-k", "candidateCount": 8, "refineCount": 2,
          "scope": "Frozen F holdout; no training, normalization fitting or Simscape; no automatic retries"})
    state = {"state": "running", "started": now(), "supervisorPid": os.getpid(),
             "completedTasks": [], "workerFailures": []}
    publish = lambda: write(output / "pipeline_status.json", state)
    quote = lambda value: str(value).replace("\\", "/").replace("'", "''")

    def task_job(task):
        assert_source(project, revision)
        destination = output / "tasks" / task["taskId"]
        script = output / f"run_{task['index']:02d}.m"
        script.write_text(f"addpath('{quote(project / 'research')}');\n"
                          f"addpath('{quote(project / 'research/remote')}');\n"
                          f"run_abfd_f_holdout_case('{quote(inputs)}','{quote(destination)}',{task['index']});\n",
                          encoding="ascii")
        run_process(["D:/matlab2019a/bin/matlab.exe", "-singleCompThread", "-batch", f"run('{quote(script)}')"],
                    project, output / "jobs" / task["taskId"])
        status = read(destination / "task_status.json")
        if status["state"] != "complete" or status["taskId"] != task["taskId"]:
            raise RuntimeError("Missing native task completion")
        return task["taskId"]

    publish()
    try:
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(task_job, task): task for task in tasks}
            for future in as_completed(futures):
                try:
                    state["completedTasks"].append(future.result())
                except Exception as error:
                    state["workerFailures"].append({"taskId": futures[future]["taskId"], "error": str(error)})
                publish()
        state["state"] = "failed" if state["workerFailures"] else "complete"
    except Exception as error:
        state.update(state="failed", message=str(error))
    state["finished"] = now()
    publish()
    return 0 if state["state"] == "complete" else 1


if __name__ == "__main__":
    sys.exit(main())
