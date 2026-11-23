"""Plumbing fixtures test strict acceptance/identity, never train a model."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "remote"))
from run_f_batch_pipeline import merge_batch, read, write, sha, task_order


class BatchContracts(unittest.TestCase):
    def prepare(self, output):
        tasks = [{"index": i, "taskId": f"task_{i}", "split": s, "taskHash": f"hash{i}"}
                 for i, s in enumerate(("train", "validation", "test", "train"), 1)]
        manifest = {"tasks": tasks, "candidateCount": 2, "configHash": "config",
                    "implementationHash": "planner", "generatorHash": "generator"}
        write(output / "task_manifest.json", manifest)
        for task in tasks:
            folder = output / "tasks" / task["taskId"]
            passed = task["index"] < 4
            attempts = [{"taskId": task["taskId"], "start": j, "passed": passed and j == 1, "reason": "fixture"}
                        for j in (1, 2)]
            write(folder / "F1/proposal_attempts.json", attempts)
            status = dict(state="complete", taskId=task["taskId"], taskHash=task["taskHash"],
                          configHash="config", implementationHash="planner", passed=int(passed), attempted=2, datasetHash="")
            if passed:
                data = dict(x=[0., 1.], y=[0.]*24, groups=[task["taskId"]], split=[task["split"]],
                            objective=1., routeMode=["base"], startIndex=1, candidateSeconds=5.,
                            solveSeconds=2., iterations=3, retryCount=0, implementationHash="generator")
                file = folder / "F1/proposal_dataset.json"
                write(file, data)
                status["datasetHash"] = sha(file)
            write(folder / "task_status.json", status)
        return tasks

    def test_complete_merge_includes_zero_success_tasks(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            tasks = self.prepare(output)
            report = merge_batch(output)
            self.assertEqual((report["tasks"], report["attempts"], report["accepted"]), (4, 8, 3))
            dataset = read(output / "proposal_dataset.json")
            self.assertEqual(dataset["frozenTasks"], tasks)
            self.assertEqual(len(dataset["y"][0]), 24)
            self.assertEqual([t["index"] for t in task_order(tasks)], [1, 2, 3, 4])

    def test_incomplete_or_mixed_shard_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            self.prepare(output)
            file = output / "tasks/task_2/task_status.json"
            original = read(file)
            for field in ("taskHash", "configHash", "implementationHash", "state", "datasetHash"):
                changed = copy.deepcopy(original)
                changed[field] = "wrong"
                write(file, changed)
                with self.assertRaises(ValueError):
                    merge_batch(output)
            write(file, original)
            file.unlink()
            with self.assertRaises(FileNotFoundError):
                merge_batch(output)

    def test_no_status_laundering_or_duplicate_start(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            self.prepare(output)
            file = output / "tasks/task_1/F1/proposal_attempts.json"
            original = read(file)
            changed = copy.deepcopy(original)
            changed[1]["passed"] = True
            write(file, changed)
            with self.assertRaisesRegex(ValueError, "count mismatch"):
                merge_batch(output)
            changed = copy.deepcopy(original)
            changed[1]["start"] = 1
            write(file, changed)
            with self.assertRaisesRegex(ValueError, "duplicate"):
                merge_batch(output)

    def test_wrong_split_rejected_even_when_hash_matches(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            self.prepare(output)
            file = output / "tasks/task_2/F1/proposal_dataset.json"
            data = read(file)
            data["split"] = ["train"]
            write(file, data)
            status_file = output / "tasks/task_2/task_status.json"
            status = read(status_file)
            status["datasetHash"] = sha(file)
            write(status_file, status)
            with self.assertRaisesRegex(ValueError, "split mismatch"):
                merge_batch(output)


if __name__ == "__main__":
    unittest.main()
