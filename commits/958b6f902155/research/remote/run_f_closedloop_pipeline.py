"""Execute available frozen F holdout tasks locally, one MATLAB at a time."""
import argparse
import os
from pathlib import Path
import sys

from run_f_batch_pipeline import assert_source, now, read, run_process, sha, write


def dependencies(project):
    """Include ignored MEX binaries in the freeze, not just tracked source."""
    project = Path(project)
    casadi = project / "lib/casadi-windows-matlabR2018b-v3.7.2"
    hsl = project / "lib/CoinHSL-archive.v2023.11.17.x86_64-w64-mingw32-libgfortran5/bin"
    required = [casadi / "casadiMEX.mexw64", casadi / "libcasadi.dll", casadi / "libipopt-3.dll",
                hsl / "libhsl.dll", project / "matlab/stewart_platform_model.slx",
                project / "mat/conf_simscape.mat"]
    for file in required:
        if not file.is_file():
            raise FileNotFoundError(f"Missing frozen runtime dependency: {file}")
    binaries = sorted(set(required + list(casadi.glob("*.dll")) + list(hsl.glob("*.dll"))))
    return [{"path": file.relative_to(project).as_posix(), "sha256": sha(file),
             "bytes": file.stat().st_size} for file in binaries]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--snapshot", type=Path, action="append", required=True)
    parser.add_argument("--matlab", default="D:/MATLAB/R2025b/bin/matlab.exe")
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    revision = assert_source(project)
    runtime = dependencies(project)
    inputs, output = args.input.resolve(), args.output.resolve()
    original = read(inputs / "task_manifest.json")
    tasks = [task for task in original["tasks"] if task["split"] == "test"]
    expected = [task["taskId"] for task in tasks]
    if len(expected) != 12 or len(set(expected)) != 12:
        raise ValueError("All twelve pre-training test tasks are required")
    catalog = {}
    for path in args.snapshot:
        path = path.resolve()
        transfer = read(path / "transfer_manifest.json")
        if transfer["state"] != "complete":
            raise ValueError("Transfer is incomplete")
        for task_id in transfer["taskIds"]:
            if task_id not in expected or task_id in catalog:
                raise ValueError("Unknown or duplicated transfer task")
            catalog[task_id] = {"path": str(path), "sha256": sha(path / "transfer_manifest.json")}
    output.mkdir(parents=True, exist_ok=False)
    freeze = {"revision": revision, "inputManifestHash": sha(inputs / "task_manifest.json"),
              "runnerHash": sha(__file__), "expectedTasks": expected, "transfers": catalog,
              "matlab": args.matlab, "runtimeDependencies": runtime,
              "physicalCasesPerTask": 12, "methodsPerCase": 3,
              "timingProtocol": "One local MATLAB worker at a time; no parallel A solver during this batch",
              "scope": "Available input wave only; absent original tasks remain pending, never deleted from the denominator"}
    write(output / "source_manifest.json", freeze)
    state = {"state": "running", "started": now(), "supervisorPid": os.getpid(),
             "expectedTasks": expected, "completedTasks": [], "workerFailures": [],
             "awaitingTransfer": [task_id for task_id in expected if task_id not in catalog]}
    publish = lambda: write(output / "pipeline_status.json", state)
    quote = lambda value: str(value).replace("\\", "/").replace("'", "''")
    publish()
    for task_id in expected:
        if task_id not in catalog:
            continue
        try:
            assert_source(project, revision)
            if dependencies(project) != runtime:
                raise ValueError("Frozen runtime dependencies changed")
            snapshot = Path(catalog[task_id]["path"])
            if sha(snapshot / "transfer_manifest.json") != catalog[task_id]["sha256"]:
                raise ValueError("Frozen transfer changed")
            destination = output / "tasks" / task_id
            script = output / (task_id + ".m")
            script.write_text(f"addpath('{quote(project / 'research')}');\n"
                              f"addpath('{quote(project / 'research/remote')}');\n"
                              f"run_abfd_f_closedloop_task('{quote(inputs)}','{quote(snapshot)}',"
                              f"'{task_id}','{quote(destination)}');\n", encoding="ascii")
            run_process([args.matlab, "-wait", "-singleCompThread", "-batch", f"run('{quote(script)}')"],
                        project, output / "jobs" / task_id)
            result = read(destination / "status.json")
            if result["state"] != "complete" or result["taskId"] != task_id or result["plannedCount"] != 36:
                raise RuntimeError("Missing complete native F closed-loop task record")
            state["completedTasks"].append({"taskId": task_id, "simulatedCount": result["simulatedCount"],
                                           "acceptedCount": result["acceptedCount"],
                                           "planningRejectedCount": result["planningRejectedCount"],
                                           "executionFailedCount": result["executionFailedCount"]})
        except Exception as error:
            state["workerFailures"].append({"taskId": task_id, "message": str(error)})
        publish()
    state["state"] = "failed" if state["workerFailures"] else ("partial" if state["awaitingTransfer"] else "complete")
    state["finished"] = now()
    publish()
    return 1 if state["workerFailures"] else 0


if __name__ == "__main__":
    sys.exit(main())
