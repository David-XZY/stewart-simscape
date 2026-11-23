"""Transfer guards use tiny file fixtures, never training models or solver output."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "remote"))
from run_f_closedloop_pipeline import dependencies
from snapshot_f_holdout import read, sha, snapshot
from summarize_f_closedloop import CASE_IDS, METHODS, summarize


class TransferTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        self.source.mkdir()
        self.ids = [f"task_{index:08d}" for index in range(12)]
        self.put("source_manifest.json", {"tasks": [{"taskId": task_id} for task_id in self.ids]})
        self.put("pipeline_status.json", {"state": "running", "completedTasks": self.ids[:1]})
        self.prefix = f"tasks/{self.ids[0]}"
        self.status = {"state": "complete", "taskId": self.ids[0], "methods": [
            {"method": "geometric-single", "passed": True},
            {"method": "nearest", "passed": False}, {"method": "mlp", "passed": False}]}
        self.put(self.prefix + "/task_status.json", self.status)
        self.put(self.prefix + "/manifest.json", {"fixture": True})
        self.put(self.prefix + "/warmstart_benchmark.json", {"fixture": True})
        for method, name in ((1, "best_result.mat"), (2, "ranking.mat"), (3, "ranking.mat")):
            self.put(f"{self.prefix}/{self.ids[0]}/method_{method:02d}/{name}", {"fixture": True})

    def put(self, relative, value):
        file = self.source / relative
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(json.dumps(value), encoding="utf-8")

    def test_completed_snapshot_keeps_failed_methods_and_all_original_tasks(self):
        output = self.root / "delivery"
        result = snapshot(self.source, output)
        self.assertEqual(result["expectedTaskIds"], self.ids)
        self.assertEqual(result["taskIds"], self.ids[:1])
        self.assertEqual(read(output / self.ids[0] / "task_status.json"), self.status)
        for entry in result["files"]:
            self.assertEqual(sha(output / entry["path"]), entry["sha256"])
        self.assertEqual(sum(entry["path"].endswith("best_result.mat") for entry in result["files"]), 1)

    def test_supervisor_completion_cannot_replace_native_completion(self):
        self.status["state"] = "running"
        self.put(self.prefix + "/task_status.json", self.status)
        with self.assertRaisesRegex(ValueError, "native"):
            snapshot(self.source, self.root / "delivery")
        self.assertFalse((self.root / "delivery").exists())

    def test_previous_snapshot_does_not_reexecute_task(self):
        snapshot(self.source, self.root / "first")
        with self.assertRaisesRegex(ValueError, "No newly completed"):
            snapshot(self.source, self.root / "second", [self.root / "first"])
        self.assertFalse((self.root / "second").exists())

    def test_unsafe_task_identity_rejected_before_copy(self):
        self.ids[0] = "../escaped"
        self.put("source_manifest.json", {"tasks": [{"taskId": task_id} for task_id in self.ids]})
        with self.assertRaisesRegex(ValueError, "identities"):
            snapshot(self.source, self.root / "delivery")

    def test_missing_mex_and_changed_runtime_detected(self):
        project = self.root / "runtime"
        relative = ["lib/casadi-windows-matlabR2018b-v3.7.2/" + name
                    for name in ("casadiMEX.mexw64", "libcasadi.dll", "libipopt-3.dll")]
        relative += ["lib/CoinHSL-archive.v2023.11.17.x86_64-w64-mingw32-libgfortran5/bin/libhsl.dll",
                     "matlab/stewart_platform_model.slx", "mat/conf_simscape.mat"]
        for name in relative[1:]:
            file = project / name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(b"dependency fixture only")
        with self.assertRaisesRegex(FileNotFoundError, "casadiMEX"):
            dependencies(project)
        (project / relative[0]).write_bytes(b"mex fixture only")
        before = dependencies(project)
        (project / relative[1]).write_bytes(b"changed dependency fixture")
        self.assertNotEqual(before, dependencies(project))


class SummaryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.inputs = self.root / "inputs"
        self.wave = self.root / "wave"
        self.ids = [f"task_{index:08d}" for index in range(12)]
        self.write(self.inputs / "task_manifest.json", {"tasks": [{"taskId": task, "split": "test"} for task in self.ids]})
        (self.inputs / "frozen.mat").write_bytes(b"input freeze fixture")
        self.write(self.wave / "source_manifest.json", {
            "inputManifestHash": sha(self.inputs / "task_manifest.json"), "expectedTasks": self.ids,
            "physicalCasesPerTask": 12, "methodsPerCase": 3, "revision": "fixture", "runnerHash": "fixture",
            "matlab": "fixture", "transfers": {self.ids[0]: {"sha256": "fixture"}}})
        self.write(self.wave / "pipeline_status.json", {"state": "partial", "workerFailures": []})
        self.task_root = self.wave / "tasks" / self.ids[0]
        self.write(self.task_root / "execution_freeze.json", {
            "taskId": self.ids[0], "inputFreezeHash": sha(self.inputs / "frozen.mat"),
            "cases": [{"id": case} for case in CASE_IDS], "methods": list(METHODS),
            "transferManifestHash": "fixture", "sourceFiles": ["fixture"], "sourceHashes": ["fixture"]})
        self.rows = [{"taskId": self.ids[0], "caseId": case, "method": method, "status": "planning-rejected",
                      "passed": False, "planningPassed": False, "metrics": {}, "reason": "fixture rejection"}
                     for case in CASE_IDS for method in METHODS]
        self.publish()

    @staticmethod
    def write(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value), encoding="utf-8")

    def publish(self):
        self.write(self.task_root / "status.json", {
            "state": "complete", "taskId": self.ids[0], "plannedCount": 36, "records": self.rows,
            "simulatedCount": sum(row["status"] == "experiment-completed" for row in self.rows),
            "planningRejectedCount": sum(row["status"] == "planning-rejected" for row in self.rows),
            "executionFailedCount": 0, "acceptedCount": sum(row["passed"] for row in self.rows)})

    def test_partial_wave_retains_432_denominator_and_rejects_duplicate_task(self):
        result = summarize(self.inputs, [self.wave])
        self.assertEqual(result["plannedCount"], 432)
        self.assertEqual(result["states"], {"planning-rejected": 36, "pending": 396})
        self.assertEqual(result["state"], "partial")
        with self.assertRaisesRegex(ValueError, "repeated task"):
            summarize(self.inputs, [self.wave, self.wave])

    def test_failed_planning_cannot_be_reported_as_accepted(self):
        self.rows[0]["passed"] = True
        self.publish()
        with self.assertRaisesRegex(ValueError, "non-completed"):
            summarize(self.inputs, [self.wave])

    def test_completed_row_requires_unchanged_native_artifacts(self):
        row = self.rows[0]
        row.update(status="experiment-completed", passed=True, planningPassed=True)
        folder = self.task_root / row["caseId"] / row["method"]
        folder.mkdir(parents=True)
        (folder / "result.mat").write_bytes(b"simulation result fixture")
        (folder / "reference_bundle.mat").write_bytes(b"reference fixture")
        details = {name: row[name] for name in ("taskId", "caseId", "method", "passed", "metrics")}
        details.update(resultHash=sha(folder / "result.mat"), referenceHash=sha(folder / "reference_bundle.mat"),
                       acceptance={"engineeringPassed": True, "budgetPassed": True, "onlinePassed": True,
                                   "tubeCoverage95": None, "tubeApplicable": False, "passed": True})
        self.write(folder / "manifest.json", {"status": "experiment-completed", "details": details})
        self.publish()
        self.assertEqual(summarize(self.inputs, [self.wave])["acceptedCount"], 1)
        (folder / "result.mat").write_bytes(b"changed result fixture")
        with self.assertRaisesRegex(ValueError, "changed"):
            summarize(self.inputs, [self.wave])


if __name__ == "__main__":
    unittest.main()
