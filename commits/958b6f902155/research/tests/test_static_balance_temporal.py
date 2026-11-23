"""Toy observations test conditioning and leakage rejection, not trained models."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np

import test_temporal_model as base_tests
from test_temporal_model import MODULE, ROOT

SPEC = importlib.util.spec_from_file_location("balance", ROOT / "research/b_residual_model/static_balance_context.py")
BALANCE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BALANCE)


class StaticBalanceTests(base_tests.TemporalTests):
    def setUp(self):
        super().setUp()
        self.clean_y = np.asarray(self.data["y"]).copy()
        q = np.array([0., 0., 1., 0., 0., 0.])
        self.data["x"] = np.tile(np.r_[q, np.zeros(6)], (900, 1)).tolist()
        a = np.deg2rad([-15, 15, 105, 135, 225, 255])
        b = np.deg2rad([45, 75, 165, 195, 285, 315])
        base = .75*np.stack((np.cos(a), np.sin(a), np.zeros(6)))
        top = .5*np.stack((np.cos(b), np.sin(b), np.zeros(6)))
        model = {"A": base.tolist(), "B": top.tolist(), "legMap": [6, 1, 2, 3, 4, 5],
                 "num": {"rcondMin": 1e-10}, "g": [0., 0., -9.81],
                 "dynamics": {"totalMass": 100., "comP": [0., 0., 0.], "includeGravity": True}}
        self.net["physical"] = {"mass": 100., "com": [0., 0., 0.],
                                "elasticity": {"reference": "initial-leg-length-v1"}}
        self.net["referencePhysical"] = {"mass": 100., "com": [0., 0., 0.]}
        self.contexts, self.tasks, sources, elastic = [], [], [], []
        values = self.clean_y.copy()
        for index, group in enumerate(dict.fromkeys(self.data["groups"])):
            ids = np.flatnonzero(np.asarray(self.data["groups"]) == group)
            kind = self.data["split"][ids[0]]
            force = np.arange(6.)+200+10*index
            # Independent closed-form oracle at identity orientation, zero COM.
            rb = top[:, [5, 0, 1, 2, 3, 4]].T
            legs = q[:3]+rb-base.T
            unit = legs/np.linalg.norm(legs, axis=1)[:, None]
            expected = np.r_[np.sum(unit*force[:, None], axis=0),
                             np.sum(np.cross(rb, unit)*force[:, None], axis=0)]-[0, 0, 981., 0, 0, 0]
            np.testing.assert_allclose(BALANCE.independent_offset(self.net, model, q, force), expected, atol=1e-12)
            values[ids] += expected
            calibration = {"state": "complete", "passed": True, "pose": q.tolist(),
                           "equilibriumForce": force.tolist(), "plantHash": "a"*64,
                           "initialAccelerationAfter": [0.]*6}
            path = self.path / (group+"_observation.json")
            path.write_text(json.dumps(calibration), encoding="utf-8")
            self.contexts.append({"taskId": group, "split": kind, "initialPose": q.tolist(),
                                  "staticForce": force.tolist(), "wrenchOffset": expected.tolist(),
                                  "calibrationFile": str(path), "calibrationHash": BALANCE.sha(path),
                                  "plantHash": "a"*64, "sourceHash": "b"*64,
                                  "collectionSeconds": 1., "reusedObservation": False})
            sources.append({"taskId": group, "split": kind, "resultHash": "b"*64})
            elastic.append({"taskId": group, "initialPose": q.tolist(), "sourceHash": "b"*64})
            self.tasks.append({"taskId": group, "split": kind, "model": copy.deepcopy(model)})
        self.data.update(y=values.tolist(), pairedSourceCases=sources, elasticityContexts=elastic)
        self.timed = dict(self.data, time=self.timed["time"])
        self.data["time"] = self.timed["time"]

    def write_conditional(self):
        model, original, timed = self.write()
        tasks = self.path / "tasks.json"
        tasks.write_text(json.dumps(self.tasks), encoding="utf-8")
        self.timed["initialBalanceContexts"] = self.contexts
        self.timed["conditioning"] = {"kind": BALANCE.KIND, "staticAssetHash": BALANCE.sha(model),
                                      "baseDatasetHash": BALANCE.sha(original)}
        timed.write_text(json.dumps(self.timed), encoding="utf-8")
        state = {"state": "complete", "passed": True, "completed": 6, "expected": 6,
                 "staticAssetHash": BALANCE.sha(model), "baseDatasetHash": BALANCE.sha(original),
                 "contextDatasetHash": BALANCE.sha(timed), "tasksHash": BALANCE.sha(tasks),
                 "protocol": BALANCE.KIND, "cases": self.contexts}
        (self.path / "status.json").write_text(json.dumps(state), encoding="utf-8")
        return model, original, timed, tasks

    def test_observed_offset_is_subtracted_before_fitting(self):
        conditioned = MODULE.fit(*self.write_conditional())
        self.timed.pop("initialBalanceContexts"); self.timed.pop("conditioning")
        self.data["y"] = self.clean_y.tolist(); self.timed["y"] = self.clean_y.tolist()
        baseline = MODULE.fit(*self.write())
        for field in ("transition", "processCovariance", "initialCovariance", "standardizedMean", "validationScale"):
            np.testing.assert_allclose(conditioned[field], baseline[field], rtol=1e-9, atol=1e-9)
        self.assertEqual(conditioned["order"], baseline["order"])
        self.assertEqual(conditioned["conditioning"]["contexts"], 6)

    def test_test_observation_cannot_change_fitted_model(self):
        first = MODULE.fit(*self.write_conditional())
        context = self.contexts[-1]
        force = np.asarray(context["staticForce"])+3
        context["staticForce"] = force.tolist()
        context["wrenchOffset"] = BALANCE.independent_offset(self.net, self.tasks[-1]["model"], context["initialPose"], force).tolist()
        path = Path(context["calibrationFile"])
        calibration = BALANCE.read(path); calibration["equilibriumForce"] = force.tolist()
        path.write_text(json.dumps(calibration), encoding="utf-8")
        context["calibrationHash"] = BALANCE.sha(path)
        second = MODULE.fit(*self.write_conditional())
        for field in ("transition", "processCovariance", "initialCovariance", "standardizedMean", "order", "validationScale"):
            np.testing.assert_array_equal(first[field], second[field])
        self.assertNotEqual(first["evaluation"]["test"], second["evaluation"]["test"])

    def test_missing_manifest_rejected(self):
        paths = self.write_conditional()
        with self.assertRaisesRegex(ValueError, "requires the frozen task manifest"):
            MODULE.fit(*paths[:3])

    def test_duplicate_context_rejected(self):
        self.contexts[-1] = self.contexts[0]
        with self.assertRaisesRegex(ValueError, "duplicate or foreign"):
            MODULE.fit(*self.write_conditional())

    def test_offset_cannot_be_fitted_from_motion(self):
        self.contexts[0]["wrenchOffset"][2] += 10
        with self.assertRaisesRegex(ValueError, "independent wrench offset"):
            MODULE.fit(*self.write_conditional())

    def test_observation_source_mutation_rejected(self):
        paths = self.write_conditional()
        Path(self.contexts[0]["calibrationFile"]).write_text("{}", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "observation file hash"):
            MODULE.fit(*paths)

    def test_foreign_conditional_asset_rejected(self):
        paths = self.write_conditional()
        self.timed["conditioning"]["staticAssetHash"] = "f"*64
        paths[2].write_text(json.dumps(self.timed), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "foreign static asset"):
            MODULE.fit(*paths)


if __name__ == "__main__":
    unittest.main()
