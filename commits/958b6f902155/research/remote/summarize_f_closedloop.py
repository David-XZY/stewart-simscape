"""Verify and combine F waves while retaining the complete 432-case denominator."""
import argparse
from collections import Counter
from itertools import product
import json
from pathlib import Path

from run_f_batch_pipeline import now, read, sha, write

METHODS = ("geometric-single", "nearest", "mlp")
CASE_IDS = tuple(f"{case}_{observer}" for observer in ("direct", "external-observer")
                 for case in ("nominal", "payload", "actuator", "initial-error", "friction-compliance", "combined"))
TERMINAL = ("experiment-completed", "planning-rejected", "failed")


def summarize(inputs, waves):
    inputs = Path(inputs)
    original = read(inputs / "task_manifest.json")
    tasks = [task["taskId"] for task in original["tasks"] if task["split"] == "test"]
    if len(tasks) != 12 or len(set(tasks)) != 12:
        raise ValueError("All twelve original test tasks are required")
    manifest_hash = sha(inputs / "task_manifest.json")
    frozen_hash = sha(inputs / "frozen.mat")
    table = {(task, case, method): {"taskId": task, "caseId": case, "method": method,
                                  "status": "pending", "passed": False, "reason": "awaiting execution"}
             for task, case, method in product(tasks, CASE_IDS, METHODS)}
    seen = set()
    common = None
    execution_sources = None
    wave_records = []
    for wave in map(Path, waves):
        freeze = read(wave / "source_manifest.json")
        if (freeze["inputManifestHash"] != manifest_hash or freeze["expectedTasks"] != tasks
                or freeze["physicalCasesPerTask"] != 12 or freeze["methodsPerCase"] != 3):
            raise ValueError("Wave has different original inputs or comparison scope")
        identity = (freeze["revision"], freeze["runnerHash"], freeze["matlab"], freeze.get("runtimeDependencies"))
        if common is None:
            common = identity
        if common != identity:
            raise ValueError("Cannot combine different frozen execution implementations or dependencies")
        pipeline = read(wave / "pipeline_status.json")
        wave_records.append({"path": str(wave.resolve()), "state": pipeline["state"],
                             "sourceManifestHash": sha(wave / "source_manifest.json"),
                             "workerFailures": pipeline["workerFailures"]})
        for task in freeze["transfers"]:
            if task not in tasks or task in seen:
                raise ValueError("Unknown or repeated task across waves; select one declared execution")
            seen.add(task)
            root = wave / "tasks" / task
            if not (root / "execution_freeze.json").exists():
                continue  # Includes setup failures, never fabricated numerical failures.
            execution = read(root / "execution_freeze.json")
            if (execution["taskId"] != task or execution["inputFreezeHash"] != frozen_hash
                    or [case["id"] for case in execution["cases"]] != list(CASE_IDS)
                    or execution["methods"] != list(METHODS)
                    or execution["transferManifestHash"] != freeze["transfers"][task]["sha256"]):
                raise ValueError("Task execution freeze differs from the declared wave")
            implementation = (execution["sourceFiles"], execution["sourceHashes"], execution["cases"])
            if execution_sources is None:
                execution_sources = implementation
            if execution_sources != implementation:
                raise ValueError("Physical cases or execution source files changed")
            status = read(root / "status.json")
            records = status.get("records", [])
            keys = [(record["taskId"], record["caseId"], record["method"]) for record in records]
            expected = list(product([task], CASE_IDS, METHODS))
            if status["taskId"] != task or status["plannedCount"] != 36 or keys != expected:
                raise ValueError("Incomplete or duplicate declared task matrix")
            for key, record in zip(keys, records):
                if record["status"] not in (*TERMINAL, "pending", "running"):
                    raise ValueError("Unrecognized execution status")
                if record["status"] == "experiment-completed":
                    directory = root / record["caseId"] / record["method"]
                    manifest = read(directory / "manifest.json")
                    details = manifest["details"]
                    acceptance = details["acceptance"]
                    if (manifest["status"] != "experiment-completed"
                            or any(details[name] != record[name] for name in ("taskId", "caseId", "method", "passed", "metrics"))
                            or sha(directory / "result.mat") != details["resultHash"]
                            or sha(directory / "reference_bundle.mat") != details["referenceHash"]):
                        raise ValueError("Native result, reference or metric manifest changed")
                    gates = [acceptance[name] for name in ("engineeringPassed", "budgetPassed", "onlinePassed")]
                    tube = acceptance["tubeCoverage95"]
                    if acceptance["tubeApplicable"]:
                        gates.append(tube is not None and tube >= acceptance["requiredTubeCoverage"])
                    if (record["passed"] != all(gates) or record["passed"] != acceptance["passed"]
                            or not record["planningPassed"]):
                        raise ValueError("Reported acceptance disagrees with required gates")
                elif record["passed"]:
                    raise ValueError("A non-completed record cannot be accepted")
                table[key] = dict(record)
            terminal_count = sum(record["status"] in TERMINAL for record in records)
            actual_counts = {"simulatedCount": sum(record["status"] == "experiment-completed" for record in records),
                             "acceptedCount": sum(record["passed"] for record in records),
                             "executionFailedCount": sum(record["status"] == "failed" for record in records),
                             "planningRejectedCount": sum(record["status"] == "planning-rejected" for record in records)}
            if any(status[name] != count for name, count in actual_counts.items()):
                raise ValueError("Task counts disagree with native records")
            if status["state"] == "complete" and terminal_count != 36:
                raise ValueError("Task marked complete before every declared case ended")
    rows = list(table.values())
    counts = Counter(row["status"] for row in rows)
    complete = sum(counts[name] for name in TERMINAL) == 432
    summaries = []
    for method in METHODS:
        selected = [row for row in rows if row["method"] == method]
        summaries.append({"method": method, "planned": 144, "accepted": sum(row["passed"] for row in selected),
                          "states": dict(Counter(row["status"] for row in selected))})
    return {"state": "complete" if complete else "partial", "captured": now(), "plannedCount": 432,
            "acceptedCount": sum(row["passed"] for row in rows), "states": dict(counts),
            "methods": summaries, "waves": wave_records, "records": rows,
            "scope": "All 12 tasks x 12 physical/observer cases x 3 methods; verified native artifacts and declared gates, not a claim of algorithm superiority or full ABFD completion"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--wave", type=Path, action="append", required=True)
    args = parser.parse_args()
    result = summarize(args.input, args.wave)
    write(args.output, result)
    print(json.dumps({key: result[key] for key in ("state", "plannedCount", "acceptedCount", "states", "methods")}))


if __name__ == "__main__":
    main()
