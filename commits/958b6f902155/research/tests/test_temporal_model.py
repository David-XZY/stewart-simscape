"""Temporal fitting split/provenance regressions; all networks here are toy fixtures."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("temporal_fit", ROOT / "research/b_residual_model/fit_residual_temporal_model.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TemporalTests(unittest.TestCase):
    def setUp(self):
        directory = ROOT / "results/abfd/temporal_tests"
        directory.mkdir(parents=True, exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=directory)
        self.path = Path(self.temporary.name).resolve()
        assert self.path.is_relative_to(directory.resolve())
        rng = np.random.default_rng(71)
        values, groups, split, times = [], [], [], []
        for group, kind in enumerate(["train"]*4+["validation", "test"]):
            error = np.zeros((150, 6))
            for row in range(1, len(error)):
                error[row] = .8*error[row-1]+rng.normal(size=6)*.3
            values.extend(error.tolist()); times.extend((np.arange(150)*.01).tolist())
            groups.extend([f"fixture_{group}"]*150); split.extend([kind]*150)
        self.data = {"x": np.zeros((900, 12)).tolist(), "y": values, "groups": groups, "split": split}
        self.timed = dict(self.data, time=times)
        # The fitter accepts trained exports; this protocol-shaped object exists
        # only inside this temporary regression fixture and is never delivered.
        self.net = {"status": "trained", "testFixture": True, "inputMean": [0]*12,
                    "inputScale": [1]*12, "outputScale": [1]*6, "varianceCalibration": 1,
                    "layers": [{"W": np.zeros((12, 12)).tolist(), "b": [0]*12}]}

    def tearDown(self):
        assert self.path.is_relative_to((ROOT / "results/abfd/temporal_tests").resolve())
        self.temporary.cleanup()

    def write(self):
        original, timed, model = [self.path / name for name in ("original.json", "timed.json", "fixture.json")]
        original.write_text(json.dumps(self.data), encoding="utf-8")
        timed.write_text(json.dumps(self.timed), encoding="utf-8")
        self.net["datasetHash"] = hashlib.sha256(original.read_bytes()).hexdigest()
        model.write_text(json.dumps(self.net), encoding="utf-8")
        return model, original, timed

    def test_test_labels_do_not_change_fitting_selection_or_calibration(self):
        first = MODULE.fit(*self.write())
        values = np.asarray(self.data["y"])
        values[np.asarray(self.data["split"]) == "test"] *= 7
        self.data["y"] = values.tolist(); self.timed["y"] = values.tolist()
        second = MODULE.fit(*self.write())
        for field in ("transition", "processCovariance", "initialCovariance", "standardizedMean", "order", "validationScale"):
            np.testing.assert_array_equal(first[field], second[field])
        self.assertNotEqual(first["evaluation"]["test"], second["evaluation"]["test"])

    def test_original_clock_is_required(self):
        self.timed["time"][15] += .001
        with self.assertRaisesRegex(ValueError, "sample clock"):
            MODULE.fit(*self.write())

    def test_group_mutation_is_rejected(self):
        self.timed["groups"] = self.data["groups"].copy()
        self.timed["groups"][0] = "changed"
        with self.assertRaisesRegex(ValueError, "changed original groups"):
            MODULE.fit(*self.write())

    def test_foreign_static_dataset_is_rejected(self):
        model, original, timed = self.write()
        self.net["datasetHash"] = "0"*64
        model.write_text(json.dumps(self.net), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "does not match"):
            MODULE.fit(model, original, timed)

    def test_stationary_covariance_matches_scalar_solutions(self):
        covariance = MODULE.stationary_covariance(np.diag([.5, .9]), np.diag([1., 2.]))
        np.testing.assert_allclose(covariance, np.diag([1/(1-.5**2), 2/(1-.9**2)]), rtol=1e-12)
        with self.assertRaisesRegex(ValueError, "Unstable"):
            MODULE.stationary_covariance(np.eye(2), np.eye(2))


if __name__ == "__main__":
    unittest.main()
