"""Record-level regression fixtures; no trained model or trajectory acceptance."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "remote"))
from run_f_batch_pipeline import sha
from summarize_f_holdout import METHODS, summarize


class HoldoutSummaryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.inputs, self.snapshot = self.root / "inputs", self.root / "snapshot"
        self.tasks = [{"taskId": f"task_{i:08d}", "index": i, "split": "test", "taskHash": f"task-hash-{i}"}
                      for i in range(1, 13)]
        self.put(self.inputs / "task_manifest.json", {"tasks": self.tasks, "configHash": "config"})
        self.put(self.inputs / "proposal_dataset.json", {"status": "record-test-fixture"})
        for name in METHODS[1:]:
            self.put(self.inputs / "models" / name / "proposal_model.json", {"status": "record-test-fixture", "name": name})
        self.assets = [""] + [sha(self.inputs / "models" / name / "proposal_model.json") for name in METHODS[1:]]
        self.add_task(self.tasks[0])

    @staticmethod
    def put(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value), encoding="utf-8")

    def add_task(self, task):
        task_id = task["taskId"]
        manifest = dict(task, taskIndex=task["index"], configHash="config", assetHashes=self.assets,
                        datasetHash=sha(self.inputs / "proposal_dataset.json"), plannerHash="planner",
                        entryFiles=["fixture"], entryHashes=["fixture-hash"], matlab="fixture")
        records, summaries = [], []
        for name in METHODS:
            count = 1 if name == METHODS[0] else 8
            for i in range(1, count + 1):
                passed = name == "nearest" and i == 1
                records.append({"taskId": task_id, "method": name, "candidate": i, "selected": i <= min(2, count),
                                "passed": passed, "iterations": 3 if i <= 2 else 0, "seconds": 2, "objective": 1.0 if passed else None})
            summaries.append({"taskId": task_id, "method": name, "passed": name == "nearest", "candidateCount": count,
                              "refinedCount": min(2, count), "seconds": 100 if name == "geometric-single" else 10,
                              "bestObjective": 1.0 if name == "nearest" else None,
                              "diversity": {} if count == 1 else {"candidateCount": 8, "uniqueCount": 1,
                                                                 "meanPairwiseControlPointDistance": 0}})
        self.put(self.snapshot / task_id / "manifest.json", manifest)
        self.put(self.snapshot / task_id / "task_status.json", {"taskId": task_id, "state": "complete", "methods": summaries})
        self.put(self.snapshot / task_id / "warmstart_benchmark.json", {"status": "experiment-completed", "summaries": summaries,
                 "records": records, "selectionPolicy": "ranked-top-k", "candidateCount": 8, "refineCount": 2})
        self.rehash()

    def rehash(self):
        files = sorted(self.snapshot.glob("task_*/*.json"))
        self.put(self.snapshot / "transfer_manifest.json", {"state": "complete", "expectedTaskIds": [t["taskId"] for t in self.tasks],
                 "taskIds": sorted({p.parent.name for p in files}),
                 "files": [{"path": p.relative_to(self.snapshot).as_posix(), "sha256": sha(p)} for p in files]})

    def change(self, mutate):
        root = self.snapshot / self.tasks[0]["taskId"]
        file = root / "warmstart_benchmark.json"
        benchmark = json.loads(file.read_text(encoding="utf-8"))
        mutate(benchmark)
        self.put(file, benchmark)
        status = json.loads((root / "task_status.json").read_text(encoding="utf-8"))
        status["methods"] = copy.deepcopy(benchmark["summaries"])
        self.put(root / "task_status.json", status)
        self.rehash()

    def test_partial_keeps_failures_and_twelve_task_denominator(self):
        result = summarize(self.inputs, [self.snapshot])
        self.assertEqual((result["state"], result["plannedTaskCount"], result["completedTaskCount"]), ("partial", 12, 1))
        self.assertEqual(result["methods"][0]["successfulTasks"], 0)
        self.assertEqual(result["methods"][0]["methodWallSecondsIncludingFailures"]["mean"], 100)
        self.assertEqual(result["methods"][1]["successfulTasks"], 1)
        self.assertEqual(result["pairedObjectives"][0]["relativeObjectiveChange"]["count"], 0)

    def test_complete_does_not_mean_every_method_succeeded(self):
        for task in self.tasks[1:]:
            self.add_task(task)
        result = summarize(self.inputs, [self.snapshot])
        self.assertEqual(result["state"], "complete")
        self.assertEqual([m["successfulTasks"] for m in result["methods"]], [0, 12, 0])

    def test_unselected_success_rejected_after_rehash(self):
        self.change(lambda b: b["records"][1].update(selected=False))
        with self.assertRaisesRegex(ValueError, "candidates"):
            summarize(self.inputs, [self.snapshot])

    def test_incorrect_best_rejected_after_consistent_manifest_edits(self):
        self.change(lambda b: b["summaries"][1].update(bestObjective=.5))
        with self.assertRaisesRegex(ValueError, "objective"):
            summarize(self.inputs, [self.snapshot])

    def test_duplicate_snapshot_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicated task"):
            summarize(self.inputs, [self.snapshot, self.snapshot])

    def test_missing_diversity_is_not_assumed_single_candidate(self):
        self.change(lambda b: b["summaries"][1].update(diversity={}))
        with self.assertRaisesRegex(ValueError, "diversity"):
            summarize(self.inputs, [self.snapshot])

    def test_changed_asset_rejected(self):
        self.put(self.inputs / "models/nearest/proposal_model.json", {"changed": True})
        with self.assertRaisesRegex(ValueError, "assets changed"):
            summarize(self.inputs, [self.snapshot])

    def test_legacy_snapshot_keeps_original_task_universe(self):
        file = self.snapshot / "transfer_manifest.json"
        manifest = json.loads(file.read_text(encoding="utf-8"))
        del manifest["expectedTaskIds"]
        self.put(file, manifest)
        result = summarize(self.inputs, [self.snapshot])
        self.assertEqual(result["plannedTaskCount"], 12)
        self.assertEqual(result["methods"][1]["pendingTasks"], 11)
        self.assertFalse(result["provenance"][0]["transferHasTaskUniverse"])


if __name__ == "__main__":
    unittest.main()
