"""Frozen holdout membership guards; no optimization or performance fixture."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "remote"))
from run_holdout_reference_batch import check_tasks


class HoldoutReferenceTests(unittest.TestCase):
    def setUp(self):
        self.tasks = [{"taskId": f"task_{100+i}", "split": "test"} for i in range(12)]

    def test_complete_frozen_test_pool(self):
        check_tasks(self.tasks)

    def test_missing_or_duplicate_task_rejected(self):
        with self.assertRaisesRegex(ValueError, "twelve"):
            check_tasks(self.tasks[:-1])
        self.tasks[-1] = self.tasks[0]
        with self.assertRaisesRegex(ValueError, "twelve"):
            check_tasks(self.tasks)

    def test_training_group_or_path_identifier_rejected(self):
        self.tasks[0]["split"] = "train"
        with self.assertRaisesRegex(ValueError, "test groups"):
            check_tasks(self.tasks)
        self.tasks[0].update(split="test", taskId="../task_100")
        with self.assertRaisesRegex(ValueError, "identifiers"):
            check_tasks(self.tasks)


if __name__ == "__main__":
    unittest.main()
