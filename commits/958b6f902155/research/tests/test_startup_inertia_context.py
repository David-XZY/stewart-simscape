"""Manufactured mechanics and anchor rejection tests, never training assets."""
import copy
import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "b_residual_model"))
import startup_inertia_context as context


class StartupInertiaTests(unittest.TestCase):
    def setUp(self):
        q = [0., 0., 1., .1, -.2, .3]
        self.data = {"x": [q+[0.]*6]*6, "anchorX": [q+[0.]*6]*6,
                     "groups": ["a", "a", "b", "b", "c", "c"],
                     "split": ["train", "train", "validation", "validation", "test", "test"],
                     "meanAnchoring": context.ANCHOR,
                     "physical": {"inferenceCalibration": {"kind": context.KIND, "required": True}},
                     "testEvaluationRole": "historical diagnostic only",
                     "inertiaContexts": [{"taskId": g, "split": s, "initialPose": q,
                                           "firstRow": 2*i+1, "rows": 2, "exposedTest": s == "test"}
                                          for i, (g, s) in enumerate(zip("abc", ("train", "validation", "test")))]}

    def test_fixed_group_anchors(self):
        np.testing.assert_array_equal(context.anchors(self.data), self.data["anchorX"])
        self.assertIsNone(context.anchors({"x": [[0.]*12]}))

    def test_incomplete_protocol_rejected(self):
        for key in ("meanAnchoring", "anchorX", "physical"):
            data = copy.deepcopy(self.data); del data[key]
            with self.assertRaises(ValueError):
                context.anchors(data)

    def test_moving_anchor_and_velocity_rejected(self):
        for index in (0, 7):
            data = copy.deepcopy(self.data)
            data["anchorX"] = np.asarray(data["anchorX"]).copy().tolist()
            data["anchorX"][1][index] += .01
            with self.assertRaisesRegex(ValueError, "anchor"):
                context.anchors(data)

    def test_group_leakage_rejected(self):
        self.data["split"][1] = "test"
        with self.assertRaisesRegex(ValueError, "leakage"):
            context.anchors(self.data)

    def test_foreign_or_duplicate_context_rejected(self):
        self.data["inertiaContexts"][-1] = self.data["inertiaContexts"][0]
        with self.assertRaisesRegex(ValueError, "duplicate"):
            context.anchors(self.data)

    def test_exposed_test_role_required(self):
        self.data["inertiaContexts"][-1]["exposedTest"] = False
        with self.assertRaisesRegex(ValueError, "test role"):
            context.anchors(self.data)

    def test_known_nonidentity_spatial_inertia(self):
        a = np.deg2rad([-15, 15, 105, 135, 225, 255])
        b = np.deg2rad([45, 75, 165, 195, 285, 315])
        model = {"A": (.75*np.stack((np.cos(a), np.sin(a), np.zeros(6)))).tolist(),
                 "B": (.5*np.stack((np.cos(b), np.sin(b), np.zeros(6)))).tolist(),
                 "legMap": [6, 1, 2, 3, 4, 5], "num": {"rcondMin": 1e-10}}
        q = np.array(self.data["x"][0][:6]); R, _, _ = context.rotation(q, np.zeros(6))
        mass, com, inertia = 157., np.array([.01, -.02, .09]), np.diag([8., 12., 17.])
        r = R @ com; skew = context.skew(r)
        M = np.block([[mass*np.eye(3), -mass*skew],
                      [mass*skew, R @ inertia @ R.T-mass*skew @ skew]])
        J, _, _ = context.geometry(model, q)
        obs = {"pose": q.tolist(), "sensitivity": np.linalg.solve(M, J.T).tolist()}
        result = context.identify(model, obs)
        self.assertAlmostEqual(result["mass"], mass, places=10)
        np.testing.assert_allclose(result["com"], com, atol=1e-12)
        np.testing.assert_allclose(result["inertia"], inertia, atol=1e-11)
        obs["sensitivity"] = np.zeros((6, 6)).tolist()
        with self.assertRaisesRegex(ValueError, "sensitivity"):
            context.identify(model, obs)


if __name__ == "__main__":
    unittest.main()
