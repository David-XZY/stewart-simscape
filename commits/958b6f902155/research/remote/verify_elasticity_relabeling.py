"""Independently verify passive-force labels and observed initial-pose context."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def rotation(q):
    r, p, y = q[:, 3], q[:, 4], q[:, 5]
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.stack((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
                     sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
                     -sp, cp*sr, cp*cr), axis=1).reshape(-1, 3, 3)


def verify(parent_file, dataset_file, tasks_file, state_file):
    parent, data, tasks, state = map(read, (parent_file, dataset_file, tasks_file, state_file))
    sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    assert state["state"] == "complete" and state["passed"] and state["completed"] == state["expected"]
    assert sha(parent_file) == state["parentDatasetHash"] == data["inertiaDatasetHash"]
    assert sha(dataset_file) == state["datasetHash"] and sha(tasks_file) == state["tasksHash"]
    for key in ("x", "groups", "split", "time", "pairedSourceCases", "collectionAcceptance", "referencePhysical"):
        assert parent[key] == data[key], f"Frozen field changed: {key}"
    physical = dict(data["physical"])
    elastic = physical.pop("elasticity")
    assert physical == parent["physical"] and elastic == state["elasticity"]
    assert elastic["reference"] == "initial-leg-length-v1"
    train = {g for g, s in zip(data["groups"], data["split"]) if s == "train"}
    assert train == set(elastic["trainingGroups"]) == set(physical["trainingGroups"])
    stiffness, damping = elastic["stiffness"], elastic["damping"]
    assert np.isfinite([stiffness, damping]).all() and min(stiffness, damping) >= 0
    tasks = {task["taskId"]: task for task in tasks}
    cases = {case["taskId"]: case for case in parent["pairedSourceCases"]}
    x, y, old_y = (np.asarray(value, dtype=float) for value in (data["x"], data["y"], parent["y"]))
    cursor, largest, seen = 0, 0., set()
    for context in data["elasticityContexts"]:
        group, count = context["taskId"], context["rows"]
        assert group not in seen and context["firstRow"] == cursor+1 and count > 0
        seen.add(group)
        ids = slice(cursor, cursor+count)
        assert set(data["groups"][ids]) == {group} and set(data["split"][ids]) == {context["split"]}
        assert tasks[group]["split"] == context["split"] == cases[group]["split"]
        assert context["sourceHash"] == cases[group]["resultHash"]
        q, v = x[ids, :6], x[ids, 6:]
        assert np.array_equal(q[0], context["initialPose"])
        model = tasks[group]["model"]
        base = np.asarray(model["A"]).T
        top = np.asarray(model["B"])[:, np.asarray(model["legMap"], dtype=int)-1].T
        rb = np.einsum("nij,kj->nki", rotation(q), top)
        legs = q[:, None, :3]+rb-base[None, :, :]
        length = np.linalg.norm(legs, axis=2)
        unit = legs/length[:, :, None]
        assert np.allclose(length[0], context["initialLength"], rtol=0, atol=1e-10)
        pitch, yaw = q[:, 4], q[:, 5]
        omega = np.stack((np.cos(yaw)*np.cos(pitch)*v[:, 3]-np.sin(yaw)*v[:, 4],
                          np.sin(yaw)*np.cos(pitch)*v[:, 3]+np.cos(yaw)*v[:, 4],
                          -np.sin(pitch)*v[:, 3]+v[:, 5]), axis=1)
        velocity = v[:, None, :3]+np.cross(omega[:, None, :], rb)
        speed = np.sum(unit*velocity, axis=2)
        passive = stiffness*(length-length[0])+damping*speed
        # Sum force and moment directly; no MATLAB regressor or wrench helper.
        force = np.sum(unit*passive[:, :, None], axis=1)
        moment = np.sum(np.cross(rb, unit)*passive[:, :, None], axis=1)
        expected = old_y[ids]-np.concatenate((force, moment), axis=1)
        largest = max(largest, float(np.max(np.abs(expected-y[ids]))))
        cursor += count
    assert cursor == len(x) and seen == set(cases) and largest < 1e-8 and np.isfinite(y).all()
    return {"passed": True, "rows": cursor, "trainingGroups": len(train), "contexts": len(seen),
            "maxIndependentTargetError": largest, "datasetHash": sha(dataset_file),
            "parentDatasetHash": sha(parent_file), "sourceHash": sha(__file__),
            "scope": "Independent geometry, virtual leg velocity and wrench sum; observed initial pose; no held-out coefficient fit"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("parent", "dataset", "tasks", "state", "output"):
        parser.add_argument(name)
    args = parser.parse_args()
    result = verify(args.parent, args.dataset, args.tasks, args.state)
    output = Path(args.output)
    if output.exists():
        raise ValueError("Use a fresh verification file")
    output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(result))
