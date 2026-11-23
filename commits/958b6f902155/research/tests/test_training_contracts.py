import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from training_io import load_dataset
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "f_trajectory_proposal"))
from sample_trajectory_candidates import sample


class TrainingContracts(unittest.TestCase):
    def test_candidate_contract_and_untrained_rejection(self):
        task = {"context": [0., 1.], "scene": {"q0": [0.]*6, "qWaypoint": [1.]*6}}
        asset = {"kind": "trajectory-proposal", "status": "trained", "mode": "nearest",
                 "inputMean": [0., 0.], "inputScale": [1., 1.], "contexts": [[0., 1.]],
                 "features": [[float(k) for k in range(24)]]}
        candidates = sample(asset, task, 8, 19)
        self.assertEqual(len(candidates), 8)
        self.assertEqual(candidates[0][0][:3], [0.]*3)
        self.assertEqual(candidates[0][0][7:], [1.]*3)
        self.assertEqual(candidates[0][0][3:7], [0., 6., 12., 18.])
        asset["status"] = "test-fixture"
        with self.assertRaisesRegex(ValueError, "trained"):
            sample(asset, task, 8, 19)

    def test_group_leakage_rejected(self):
        data = {"schemaVersion": 1, "x": [[0], [1], [2]], "y": [[0], [1], [2]],
                "groups": ["same_task", "same_task", "heldout"],
                "split": ["train", "validation", "test"]}
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory)/"data.json"
            file.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "leakage"):
                load_dataset(file)

    def test_nearest_ties_preserve_trajectory_library_order(self):
        # Multi-start trajectories share contexts. Equidistant task groups must
        # retain library order so Python and MATLAB select the same top eight.
        task = {"context": [0., 0.], "scene": {"q0": [0.]*6, "qWaypoint": [1.]*6}}
        asset = {"kind": "trajectory-proposal", "status": "trained", "mode": "nearest",
                 "inputMean": [0., 0.], "inputScale": [1., 1.],
                 "contexts": [[-1., 0.], [1., 0.], [3., 0.]] * 8,
                 "features": [[float(index)] * 24 for index in range(24)]}
        expected = [0, 1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18, 19, 21, 22,
                    2, 5, 8, 11, 14, 17, 20, 23, 0, 1]
        for seed in (19, 20260905):
            candidates = sample(asset, task, 26, seed)
            self.assertEqual([cp[0][3] for cp in candidates], expected)
            self.assertEqual([cp[5][6] for cp in candidates], expected)

    def test_complete_grouped_dataset(self):
        data = {"schemaVersion": 1, "x": [[0], [1], [2]], "y": [[0], [1], [2]],
                "groups": ["a", "b", "c"], "split": ["train", "validation", "test"]}
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory)/"data.json"
            file.write_text(json.dumps(data), encoding="utf-8")
            self.assertEqual(len(load_dataset(file)["datasetHash"]), 64)


if __name__ == "__main__":
    unittest.main()
