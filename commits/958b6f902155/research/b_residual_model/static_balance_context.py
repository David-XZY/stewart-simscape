"""Verify independent initial-balance observations before conditioning uncertainty.

Uses NumPy geometry and static force/moment balance, without MATLAB providers,
trajectory labels or a fitted held-out inertia. The original labels stay frozen.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

KIND = "independent-initial-static-balance-v1"
ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def require(condition, message):
    if not condition:
        raise ValueError("Static balance context: " + message)


def vector(value, length):
    result = np.asarray(value, dtype=float)
    require(result.shape == (length,) and np.isfinite(result).all(), "invalid observation vector")
    return result


def independent_offset(net, model, pose, force):
    q, force = vector(pose, 6), vector(force, 6)
    require(abs(np.cos(q[4])) > 1e-5, "singular initial attitude")
    r, p, y = q[3:]
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
    base, top = np.asarray(model["A"]).T, np.asarray(model["B"]).T
    require(base.shape == top.shape == (6, 3), "invalid geometry")
    mapping = np.asarray(model["legMap"], dtype=int)-1
    require(sorted(mapping.tolist()) == list(range(6)), "invalid leg mapping")
    rb = top[mapping] @ rotation.T
    legs = q[:3]+rb-base
    length = np.linalg.norm(legs, axis=1)
    require(np.all(length > 0), "zero leg length")
    unit = legs/length[:, None]
    transmission = np.concatenate((unit, np.cross(rb, unit)), axis=1)
    require(np.isfinite(transmission).all() and np.linalg.cond(transmission) < 1/model["num"]["rcondMin"],
            "singular force transmission")
    applied = transmission.T @ force
    physical, nominal = net["physical"], model["dynamics"]
    require(physical.get("elasticity", {}).get("reference") == "initial-leg-length-v1",
            "initial-rest-length structured asset required")
    mass = physical["mass"]
    moment = mass*vector(physical["com"], 3)
    if "referencePhysical" in net:
        old = net["referencePhysical"]
        mass += nominal["totalMass"]-old["mass"]
        moment += nominal["totalMass"]*vector(nominal["comP"], 3)-old["mass"]*vector(old["com"], 3)
    require(np.isfinite(mass) and mass > 0, "invalid transferred mass")
    gravity = vector(model["g"], 3) if nominal["includeGravity"] else np.zeros(3)
    # At rest with the spring reference equal to q0, spring/damper wrench is zero.
    required = np.concatenate((-mass*gravity, np.cross(rotation @ moment, -gravity)))
    h = (np.concatenate((q, np.zeros(6)))-net["inputMean"])/net["inputScale"]
    for index, layer in enumerate(net["layers"]):
        h = np.asarray(layer["W"]) @ h + layer["b"]
        if index+1 < len(net["layers"]):
            h = np.tanh(h)
    required += h[:6]*net["outputScale"]
    return applied-required


def verify(model_file, original_file, context_file, tasks_file, root=ROOT):
    net, original, data, tasks = map(read, (model_file, original_file, context_file, tasks_file))
    state = read(Path(context_file).parent / "status.json")
    metadata = data.get("conditioning", {})
    require(metadata.get("kind") == KIND, "unsupported conditioning protocol")
    require(net["status"] == "trained" and net["datasetHash"] == sha(original_file), "foreign static dataset")
    require(metadata.get("staticAssetHash") == state.get("staticAssetHash") == sha(model_file), "foreign static asset")
    require(metadata.get("baseDatasetHash") == state.get("baseDatasetHash") == sha(original_file), "foreign base dataset")
    require(state.get("state") == "complete" and state.get("passed") is True, "collection not complete")
    require(state.get("contextDatasetHash") == sha(context_file) and state.get("tasksHash") == sha(tasks_file),
            "collection source hash mismatch")
    require(state.get("protocol") == KIND, "collection protocol mismatch")
    for field in ("x", "y", "groups", "split", "time", "pairedSourceCases", "elasticityContexts"):
        require(original[field] == data[field], "frozen field changed: " + field)
    contexts = data.get("initialBalanceContexts", [])
    require(contexts == state.get("cases"), "collection observations changed")
    x, groups, split = np.asarray(data["x"]), np.asarray(data["groups"]), np.asarray(data["split"])
    unique = set(groups)
    require(len(contexts) == len(unique) == state.get("completed") == state.get("expected"), "missing contexts")
    task_map = {task["taskId"]: task for task in tasks}
    source_map = {case["taskId"]: case for case in data["pairedSourceCases"]}
    elastic_map = {case["taskId"]: case for case in data["elasticityContexts"]}
    require(len(task_map) == len(tasks) and set(source_map) == set(elastic_map) == unique, "invalid source groups")
    offsets, seen, worst, new_costs = np.zeros((len(x), 6)), set(), 0., []
    for context in contexts:
        group = context["taskId"]
        require(group in unique and group in task_map and group not in seen, "duplicate or foreign context")
        seen.add(group)
        ids = np.flatnonzero(groups == group)
        kind = context["split"]
        require(set(split[ids]) == {kind} and kind == task_map[group]["split"] == source_map[group]["split"],
                "context split mismatch")
        pose, offset = vector(context["initialPose"], 6), vector(context["wrenchOffset"], 6)
        require(np.allclose(pose, x[ids[0], :6], rtol=0, atol=1e-10) and
                np.allclose(pose, elastic_map[group]["initialPose"], rtol=0, atol=1e-10), "initial pose mismatch")
        require(context["sourceHash"] == source_map[group]["resultHash"] == elastic_map[group]["sourceHash"],
                "trajectory source mismatch")
        calibration_path = Path(context["calibrationFile"])
        if not calibration_path.is_absolute():
            calibration_path = Path(root) / calibration_path
        require(sha(calibration_path) == context["calibrationHash"], "observation file hash mismatch")
        calibration = read(calibration_path)
        require(calibration.get("state") == "complete" and calibration.get("passed") is True, "failed initial balance")
        require(calibration["plantHash"] == context["plantHash"], "plant mismatch")
        require(np.allclose(pose, vector(calibration["pose"], 6), rtol=0, atol=1e-10), "observation pose mismatch")
        force = vector(context["staticForce"], 6)
        require(np.allclose(force, vector(calibration["equilibriumForce"], 6), rtol=0, atol=1e-10),
                "observation force mismatch")
        require(np.max(np.abs(vector(calibration["initialAccelerationAfter"], 6))) < 1e-7,
                "static balance verification failed")
        expected = independent_offset(net, task_map[group]["model"], pose, force)
        error = float(np.max(np.abs(expected-offset)))
        require(np.isfinite(error) and error < 1e-8, "independent wrench offset mismatch")
        worst = max(worst, error)
        offsets[ids] = offset
        require(np.isfinite(context["collectionSeconds"]) and context["collectionSeconds"] >= 0, "invalid collection time")
        if not context["reusedObservation"]:
            new_costs.append(context["collectionSeconds"])
    require(seen == unique, "missing contexts")
    return offsets, {"kind": KIND, "contextDatasetHash": sha(context_file), "tasksHash": sha(tasks_file),
                     "verificationImplementationHash": sha(__file__), "contexts": len(seen),
                     "maxIndependentOffsetError": worst, "newObservationWallSeconds": new_costs,
                     "scope": "Independent pre-task static balance only; no trajectory-dependent offset fit or held-out inertia fit; observation cost additional"}
