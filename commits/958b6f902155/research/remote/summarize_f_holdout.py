"""Aggregate immutable numerical F outcomes without removing failed tasks."""
import argparse
import itertools
import math
from pathlib import Path, PurePosixPath
import statistics

from run_f_batch_pipeline import now, read, sha, write

METHODS = ("geometric-single", "nearest", "mlp")


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def distribution(values):
    values = sorted(values)
    if not values:
        return {"count": 0, "mean": None, "median": None, "p95": None}
    index = .95 * (len(values) - 1)
    lower, upper = math.floor(index), math.ceil(index)
    return {"count": len(values), "mean": statistics.mean(values), "median": statistics.median(values),
            "p95": values[lower] + (index - lower) * (values[upper] - values[lower])}


def summarize(inputs, snapshots):
    inputs = Path(inputs)
    freeze = read(inputs / "task_manifest.json")
    selected_tasks = [task for task in freeze["tasks"] if task["split"] == "test"]
    tasks = {task["taskId"]: task for task in selected_tasks}
    if len(selected_tasks) != 12 or len(tasks) != 12:
        raise ValueError("All twelve original test tasks are required")
    asset_hashes = [""] + [sha(inputs / "models" / name / "proposal_model.json") for name in METHODS[1:]]
    dataset_hash = sha(inputs / "proposal_dataset.json")
    completed, rows, provenance = set(), [], []
    implementation = None
    for snapshot in map(Path, snapshots):
        transfer = read(snapshot / "transfer_manifest.json")
        # The first immutable transfer predates expectedTaskIds. The original
        # task manifest remains the authority for the complete denominator.
        if (transfer["state"] != "complete" or ("expectedTaskIds" in transfer
                and (len(transfer["expectedTaskIds"]) != 12 or set(transfer["expectedTaskIds"]) != set(tasks)))):
            raise ValueError("Incomplete transfer or a different original test set")
        paths = set()
        for entry in transfer["files"]:
            relative = PurePosixPath(entry["path"])
            if relative.is_absolute() or ".." in relative.parts or ":" in str(relative) or "\\" in str(relative):
                raise ValueError("Transfer path escapes the snapshot")
            if entry["path"] in paths or sha(snapshot / relative) != entry["sha256"]:
                raise ValueError("Duplicated or changed transfer artifact")
            paths.add(entry["path"])
        for task_id in transfer["taskIds"]:
            if task_id not in tasks or task_id in completed:
                raise ValueError("Unknown or duplicated task")
            required = {f"{task_id}/{name}" for name in ("manifest.json", "task_status.json", "warmstart_benchmark.json")}
            if not required <= paths:
                raise ValueError("Native outcomes must be covered by transfer hashes")
            root = snapshot / task_id
            manifest, status, benchmark = (read(root / name) for name in
                                           ("manifest.json", "task_status.json", "warmstart_benchmark.json"))
            task = tasks[task_id]
            if (manifest["taskId"] != task_id or manifest["taskIndex"] != task["index"]
                    or manifest["taskHash"] != task["taskHash"] or manifest["split"] != "test"
                    or manifest["configHash"] != freeze["configHash"] or manifest["datasetHash"] != dataset_hash
                    or manifest["assetHashes"] != asset_hashes):
                raise ValueError("Original task, configuration, data or trained assets changed")
            code = {key: manifest[key] for key in ("plannerHash", "entryFiles", "entryHashes", "matlab")}
            if implementation is not None and implementation != code:
                raise ValueError("Different numerical implementations cannot be pooled")
            implementation = code
            if (status["state"] != "complete" or status["taskId"] != task_id
                    or benchmark["status"] != "experiment-completed" or status["methods"] != benchmark["summaries"]
                    or [method["method"] for method in status["methods"]] != list(METHODS)
                    or benchmark["selectionPolicy"] != "ranked-top-k"
                    or benchmark["candidateCount"] != 8 or benchmark["refineCount"] != 2):
                raise ValueError("Missing native completion, method or eight-to-two policy")
            if len(benchmark["records"]) != 17:
                raise ValueError("All geometric and learned candidate outcomes must remain")
            for method in status["methods"]:
                name = method["method"]
                candidates = [row for row in benchmark["records"] if row["method"] == name]
                count = 1 if name == METHODS[0] else 8
                if (sorted(row["candidate"] for row in candidates) != list(range(1, count + 1))
                        or any(row["taskId"] != task_id or type(row["passed"]) is not bool
                               or type(row["selected"]) is not bool or (row["passed"] and not row["selected"])
                               for row in candidates)):
                    raise ValueError("Missing, duplicate or inconsistent candidates")
                selected = [row for row in candidates if row["selected"]]
                accepted = [row for row in candidates if row["passed"]]
                if (method["taskId"] != task_id or type(method["passed"]) is not bool
                        or method["passed"] != bool(accepted) or method["candidateCount"] != count
                        or method["refinedCount"] != len(selected) or len(selected) > (1 if count == 1 else 2)
                        or not finite(method["seconds"]) or method["seconds"] < 0
                        or any(not finite(row["iterations"]) or row["iterations"] < 0
                               or not finite(row["seconds"]) or row["seconds"] < 0 for row in candidates)):
                    raise ValueError("Invalid method outcome, timing or selected count")
                best = min((row["objective"] for row in accepted if finite(row["objective"])), default=None)
                if (len([row for row in accepted if finite(row["objective"])]) != len(accepted)
                        or method["bestObjective"] != best):
                    raise ValueError("Reported objective differs from the selected accepted minimum")
                if name != METHODS[0]:
                    diversity = method["diversity"]
                    unique = diversity.get("uniqueCount")
                    distance = diversity.get("meanPairwiseControlPointDistance")
                    if (diversity.get("candidateCount") != 8 or not finite(unique) or unique != int(unique)
                            or not 1 <= unique <= 8 or not finite(distance) or distance < 0):
                        raise ValueError("Missing or invalid learned-candidate diversity")
                rows.append({"taskId": task_id, "method": name, "passed": method["passed"],
                             "seconds": method["seconds"], "bestObjective": best,
                             "selectedCandidates": len(selected), "selectedIterations": sum(row["iterations"] for row in selected),
                             "diversity": method["diversity"], "candidateOutcomes": candidates})
            completed.add(task_id)
            provenance.append({"taskId": task_id, "snapshot": str(snapshot),
                               "manifestHash": sha(root / "manifest.json"), "outcomeHash": sha(root / "warmstart_benchmark.json"),
                               "taskUniverseSource": "original pre-training task manifest",
                               "transferHasTaskUniverse": "expectedTaskIds" in transfer})
    methods = []
    for name in METHODS:
        group = [row for row in rows if row["method"] == name]
        methods.append({"method": name, "plannedTasks": 12, "completedTasks": len(group),
                        "successfulTasks": sum(row["passed"] for row in group), "pendingTasks": 12 - len(group),
                        "methodWallSecondsIncludingFailures": distribution([row["seconds"] for row in group]),
                        "recordedIhsidIterationsIncludingFailures": distribution([row["selectedIterations"] for row in group]),
                        "proposalUniqueCounts": [row["diversity"].get("uniqueCount", 1) for row in group]})
    pairs = []
    for baseline, other in itertools.combinations(METHODS, 2):
        first = {row["taskId"]: row for row in rows if row["method"] == baseline and row["passed"]}
        second = {row["taskId"]: row for row in rows if row["method"] == other and row["passed"]}
        common = sorted(first.keys() & second.keys())
        pairs.append({"baseline": baseline, "other": other, "bothSucceededTaskIds": common,
                      "relativeObjectiveChange": distribution([(second[t]["bestObjective"] - first[t]["bestObjective"])
                                                               / max(abs(first[t]["bestObjective"]), 1e-12) for t in common])})
    return {"state": "complete" if len(completed) == 12 else "partial", "captured": now(),
            "plannedTaskCount": 12, "completedTaskCount": len(completed), "pendingTaskIds": sorted(set(tasks) - completed),
            "methods": methods, "pairedObjectives": pairs, "records": rows, "provenance": provenance,
            "implementation": implementation,
            "scope": "Numerical F outcomes only; byte-checked transfers and native record consistency. MATLAB candidate replay and dense reference audit remain in the closed-loop consumer. Timings are observed shared-host wall times, not isolated hardware benchmarks. Paired objectives use only jointly successful tasks; all failures remain in the overall denominator."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--snapshot", type=Path, action="append", required=True)
    args = parser.parse_args()
    write(args.output, summarize(args.input, args.snapshot))


if __name__ == "__main__":
    main()
