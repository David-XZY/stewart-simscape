"""Synthetic anchored-mean temporal tests; no fixture is a real dataset audit."""
import json
import sys
from pathlib import Path
import unittest

import numpy as np
import test_temporal_model as base

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "b_residual_model"))
import startup_inertia_context as context


class AnchoredTemporalTests(unittest.TestCase):
    def setUp(self):
        base.TemporalTests.setUp(self)
        self.clean = np.asarray(self.data["y"]).copy()
        x, a = np.zeros((900, 12)), np.zeros((900, 12))
        contexts = []
        for i, group in enumerate(dict.fromkeys(self.data["groups"])):
            ids = np.flatnonzero(np.asarray(self.data["groups"]) == group)
            x[ids, 0] = .1*i+np.sin(np.arange(len(ids))*.02)
            a[ids, 0] = .1*i
            split = self.data["split"][ids[0]]
            contexts.append({"taskId": group, "split": split, "firstRow": int(ids[0]+1),
                             "rows": len(ids), "initialPose": x[ids[0], :6].tolist(), "exposedTest": split == "test"})
        weights = np.zeros((12, 12)); weights[:6, 0] = np.arange(6.)+.5
        self.net["layers"][0]["W"] = weights.tolist()
        self.data.update(x=x.tolist(), y=(self.clean+(x-a) @ weights[:6].T).tolist(), anchorX=a.tolist(),
                         meanAnchoring=context.ANCHOR, inertiaContexts=contexts,
                         physical={"inferenceCalibration": {"kind": context.KIND, "required": True}},
                         testEvaluationRole="synthetic fixture; historical diagnostic role")
        self.timed = dict(self.data, time=self.timed["time"])
        self.net.update(meanAnchoring=context.ANCHOR, physical=self.data["physical"],
                        datasetAuditHash="a"*64, testEvaluationRole=self.data["testEvaluationRole"])

    def tearDown(self):
        base.TemporalTests.tearDown(self)

    def write(self):
        model, original, timed = base.TemporalTests.write(self)
        tasks = self.path / "fixture_tasks.json"; tasks.write_text("[]")
        self.net["datasetAudit"] = {"passed": True, "testFixture": True, "datasetHash": context.sha(original),
                                    "rows": 900, "contexts": 6, "tasksHash": context.sha(tasks),
                                    "verificationImplementationHash": "b"*64,
                                    "maxIndependentError": {"staticOffset": 0., "physical": 0., "label": 0.}}
        model.write_text(json.dumps(self.net))
        return model, original, timed, tasks

    def test_anchored_mean_removed_before_temporal_fit(self):
        actual = base.MODULE.fit(*self.write())
        # Independent zero-mean reference contains the same manufactured VAR.
        data = {key: self.data[key] for key in ("x", "y", "groups", "split")}
        data["y"] = self.clean.tolist()
        self.data = data; self.timed = dict(data, time=self.timed["time"])
        for key in ("meanAnchoring", "datasetAudit", "physical"):
            self.net.pop(key, None)
        self.net["layers"][0]["W"] = np.zeros((12, 12)).tolist()
        expected = base.MODULE.fit(*base.TemporalTests.write(self))
        for field in ("transition", "processCovariance", "initialCovariance", "standardizedMean", "validationScale"):
            np.testing.assert_allclose(actual[field], expected[field], rtol=1e-9, atol=1e-10)
        self.assertEqual(actual["conditioning"]["kind"], context.KIND)

    def test_test_labels_do_not_change_selection(self):
        first = base.MODULE.fit(*self.write())
        values = np.asarray(self.data["y"])
        values[np.asarray(self.data["split"]) == "test"] *= 7
        self.data["y"] = values.tolist(); self.timed["y"] = values.tolist()
        second = base.MODULE.fit(*self.write())
        for field in ("transition", "processCovariance", "standardizedMean", "order", "validationScale"):
            np.testing.assert_array_equal(first[field], second[field])

    def test_foreign_audit_rejected(self):
        model, original, timed, tasks = self.write()
        self.net["datasetAudit"]["datasetHash"] = "0"*64
        model.write_text(json.dumps(self.net))
        with self.assertRaisesRegex(ValueError, "audit"):
            base.MODULE.fit(model, original, timed, tasks)


if __name__ == "__main__":
    unittest.main()
