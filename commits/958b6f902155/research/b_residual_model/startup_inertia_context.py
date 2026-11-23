"""Independent NumPy audit of explicit startup inertia and anchored B labels.

The startup force/acceleration measurement is additional per-task information.
Historical test trajectories remain diagnostic; only training trajectories fit
shared parameters and only validation trajectories select a network.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

KIND = "independent-initial-inertia-static-v2"
ANCHOR = "initial-static-state-v1"


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(ok, message):
    if not ok:
        raise ValueError("Startup inertia context: " + message)


def anchors(data):
    """Portable pre-training contract; raw measurement audit is a separate gate."""
    protocol = data.get("physical", {}).get("inferenceCalibration", {})
    anchored = "meanAnchoring" in data or "anchorX" in data or bool(protocol)
    if not anchored:
        return None
    require(data.get("meanAnchoring") == ANCHOR and protocol.get("kind") == KIND
            and protocol.get("required") is True, "unsupported or incomplete mean protocol")
    x, a = np.asarray(data["x"], dtype=float), np.asarray(data.get("anchorX"), dtype=float)
    require(a.shape == x.shape and x.ndim == 2 and x.shape[1] == 12
            and np.isfinite(a).all() and np.isfinite(x).all(), "invalid anchors")
    groups, split = np.asarray(data["groups"]), np.asarray(data["split"])
    require(groups.shape == split.shape == (len(x),), "invalid grouped rows")
    contexts = data.get("inertiaContexts", [])
    require(len(contexts) == len(set(groups)), "missing or duplicate contexts")
    seen = set()
    for context in contexts:
        group = context["taskId"]
        require(group not in seen and group in groups, "duplicate or foreign group")
        seen.add(group)
        ids = np.flatnonzero(groups == group)
        require(np.all(np.diff(ids) == 1) and set(split[ids]) == {context["split"]}, "group leakage or ordering")
        require(ids[0]+1 == context["firstRow"] and len(ids) == context["rows"], "context row mismatch")
        pose = np.asarray(context["initialPose"])
        require(pose.shape == (6,) and np.allclose(a[ids], np.r_[pose, np.zeros(6)], rtol=0, atol=1e-12)
                and np.allclose(x[ids[0], :6], pose, rtol=0, atol=1e-10), "moving or foreign anchor")
        require(context.get("exposedTest") == (context["split"] == "test"), "test role changed")
    require("diagnostic" in data.get("testEvaluationRole", ""), "historical test role missing")
    return a


def skew(v):
    return np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])


def rotation(q, v):
    r, p, y = q[3:]
    cp, sp, cy, sy, cr, sr = np.cos(p), np.sin(p), np.cos(y), np.sin(y), np.cos(r), np.sin(r)
    R = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
    E = np.array([[cy*cp, -sy, 0.], [sy*cp, cy, 0.], [-sp, 0., 1.]])
    Ed = np.array([[-sy*cp*v[5]-cy*sp*v[4], -cy*v[5], 0.],
                   [cy*cp*v[5]-sy*sp*v[4], -sy*v[5], 0.], [-cp*v[4], 0., 0.]])
    require(abs(cp) > 1e-5, "singular attitude")
    return R, E, Ed


def geometry(model, q):
    R, E, _ = rotation(q, np.zeros(6))
    indices = np.asarray(model["legMap"], dtype=int)-1
    require(sorted(indices.tolist()) == list(range(6)), "invalid leg map")
    rb = np.asarray(model["B"]).T[indices] @ R.T
    legs = q[:3]+rb-np.asarray(model["A"]).T
    length = np.linalg.norm(legs, axis=1)
    require(np.all(length > 0), "zero leg length")
    unit = legs/length[:, None]
    J = np.c_[unit, np.cross(rb, unit)]
    require(np.linalg.cond(J) < 1/model["num"]["rcondMin"], "singular transmission")
    T = np.eye(6); T[3:, 3:] = E
    return J, J @ T, length


def identify(model, observation):
    q = np.asarray(observation["pose"], dtype=float)
    S = np.asarray(observation["sensitivity"], dtype=float)
    require(S.shape == (6, 6) and np.isfinite(S).all() and np.linalg.cond(S) < 1e8, "invalid sensitivity")
    J, _, _ = geometry(model, q)
    M = np.linalg.solve(S.T, J).T
    require(np.linalg.norm(M-M.T)/np.linalg.norm(M) < 1e-8
            and np.linalg.eigvalsh((M+M.T)/2).min() > 0, "invalid measured mass matrix")
    R, E, _ = rotation(q, np.zeros(6))
    T = np.eye(6); T[3:, 3:] = E
    # Independent spatial-mass basis, avoiding the MATLAB wrench regressor.
    basis = []
    for k in range(10):
        block = np.zeros((6, 6))
        if k == 0:
            block[:3, :3] = np.eye(3)
        elif k < 4:
            h = R[:, k-1]
            block[:3, 3:] = -skew(h); block[3:, :3] = skew(h)
        else:
            i, j = ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))[k-4]
            I = np.zeros((3, 3)); I[i, j] = I[j, i] = 1.
            block[3:, 3:] = R @ I @ R.T
        basis.append((block @ T).ravel(order="F"))
    design = np.asarray(basis).T
    beta, _, rank, _ = np.linalg.lstsq(design, (M @ T).ravel(order="F"), rcond=None)
    mass = beta[0]; com = beta[1:4]/mass
    origin = np.array([[beta[4], beta[7], beta[8]], [beta[7], beta[5], beta[9]], [beta[8], beta[9], beta[6]]])
    inertia = origin-mass*(com@com*np.eye(3)-np.outer(com, com))
    residual = np.linalg.norm(design @ beta-(M @ T).ravel(order="F"))/np.linalg.norm(M @ T)
    require(rank == 10 and mass > 0 and np.linalg.norm(com) < .5 and residual < .01
            and np.linalg.eigvalsh(inertia).min() > 0
            and np.linalg.eigvalsh(np.trace(inertia)/2*np.eye(3)-inertia).min() >= -1e-9, "nonphysical identification")
    return {"mass": mass, "com": com, "inertia": inertia}


def wrench(model, physical, q, v, a):
    R, E, Ed = rotation(q, v)
    w, alpha = E @ v[3:], E @ a[3:]+Ed @ v[3:]
    r = R @ np.asarray(physical["com"])
    I = R @ np.asarray(physical["inertia"]) @ R.T
    g = np.asarray(model["g"]) if model["dynamics"]["includeGravity"] else np.zeros(3)
    force = physical["mass"]*(a[:3]+np.cross(alpha, r)+np.cross(w, np.cross(w, r))-g)
    return np.r_[force, I @ alpha+np.cross(w, I @ w)+np.cross(r, force)]


def verify(dataset_file, parent_file, context_file, tasks_file, raw_file):
    data, parent, old_context, tasks, raw = map(read, (dataset_file, parent_file, context_file, tasks_file, raw_file))
    anchors(data)
    status = read(Path(dataset_file).parent / "status.json")
    require(status["state"] == "complete" and status["passed"] is True
            and status["datasetHash"] == sha(dataset_file), "unaccepted dataset export")
    require(data["parentDatasetHash"] == sha(parent_file) and data["contextSourceHash"] == sha(context_file)
            and status["tasksHash"] == sha(tasks_file), "foreign parent or tasks")
    require(raw["datasetHash"] == sha(dataset_file), "foreign raw observations")
    for field in ("x", "groups", "split", "time", "pairedSourceCases", "elasticityContexts", "collectionAcceptance"):
        require(data[field] == parent[field], "frozen field changed: " + field)
    physical = dict(data["physical"]); physical.pop("inferenceCalibration")
    require(physical == parent["physical"], "shared physical parameters changed")
    groups, split, x = np.asarray(data["groups"]), np.asarray(data["split"]), np.asarray(data["x"])
    require(set(physical["trainingGroups"]) == set(groups[split == "train"]), "physical fit leakage")
    task_map = {t["taskId"]: t for t in tasks}
    old_map = {c["taskId"]: c for c in old_context["initialBalanceContexts"]}
    raw_map = {c["taskId"]: c for c in raw["cases"]}
    require(len(raw_map) == len(raw["cases"]) == len(set(groups)), "raw groups missing or duplicate")
    maximum = {"physical": 0., "staticOffset": 0., "label": 0.}
    for context in data["inertiaContexts"]:
        group = context["taskId"]; task, old, logged = task_map[group], old_map[group], raw_map[group]
        ids = np.flatnonzero(groups == group)
        require(task["split"] == context["split"] == old["split"], "frozen task split changed")
        path = Path(context["calibrationFile"])
        require(sha(path) == context["calibrationHash"] == old["calibrationHash"], "observation changed")
        obs = read(path)
        require(obs["state"] == "complete" and obs["passed"] is True
                and obs["plantHash"] == context["plantHash"]
                and obs["modelHash"] == context["physical"]["nominalModelHash"]
                and np.max(np.abs(obs["initialAccelerationAfter"])) < 1e-7, "unaccepted startup observation")
        measured = identify(task["model"], obs)
        for key in measured:
            error = float(np.max(np.abs(np.asarray(measured[key])-context["physical"][key])))
            require(error < 1e-8, "independent inertia differs")
            maximum["physical"] = max(maximum["physical"], error)
        q0 = np.asarray(context["initialPose"])
        require(np.allclose(q0, obs["pose"], rtol=0, atol=1e-10)
                and np.array_equal(context["staticForce"], obs["equilibriumForce"]), "observation context changed")
        J0, _, L0 = geometry(task["model"], q0)
        offset = J0.T @ obs["equilibriumForce"]-wrench(task["model"], measured, q0, np.zeros(6), np.zeros(6))
        error = float(np.max(np.abs(offset-context["staticOffset"])))
        require(error < 1e-8 and np.allclose(L0, context["initialLength"], rtol=0, atol=1e-10), "static anchor differs")
        maximum["staticOffset"] = max(maximum["staticOffset"], error)
        require(logged["sourceHash"] == context["sourceHash"] and logged["rows"] == len(ids), "raw source changed")
        require(sha(logged["sourceFile"]) == logged["sourceHash"], "raw replay file changed")
        require(np.array_equal(logged["x"], x[ids]) and np.array_equal(logged["time"], np.asarray(data["time"])[ids]), "raw states or clock changed")
        acceleration, actual = np.asarray(logged["qdd"]), np.asarray(logged["actualForce"])
        ft, t = np.asarray(logged["forceTime"]), np.asarray(logged["time"])
        require(logged["forceSource"] == "measured-actuator-output" and actual.shape == (len(ft), 6)
                and np.isfinite(ft).all() and np.all(np.diff(ft) > 0)
                and ft[0] <= t[0] and ft[-1] >= t[-1], "actual-force clock invalid")
        actual = np.column_stack([np.interp(t, ft, actual[:, j]) for j in range(6)])
        require(acceleration.shape == actual.shape == (len(ids), 6) and np.isfinite(acceleration).all()
                and np.isfinite(actual).all(), "invalid acceleration or actual force")
        e = physical["elasticity"]
        for j, index in enumerate(ids):
            q, v = x[index, :6], x[index, 6:]
            J, Jq, L = geometry(task["model"], q)
            expected = J.T @ (actual[j]-e["stiffness"]*(L-L0)-e["damping"]*(Jq @ v))-wrench(task["model"], measured, q, v, acceleration[j])-offset
            error = float(np.max(np.abs(expected-data["y"][index])))
            require(error < 1e-8, "independent actual-force label differs")
            maximum["label"] = max(maximum["label"], error)
    return {"state": "complete", "passed": True, "kind": KIND, "datasetHash": sha(dataset_file),
            "tasksHash": sha(tasks_file), "rawObservationsHash": sha(raw_file), "verificationImplementationHash": sha(__file__),
            "contexts": len(raw_map), "rows": len(x), "maxIndependentError": maximum,
            "scope": "NumPy reconstruction from independent startup and frozen actual-force logs; no closed-loop acceptance; historical test diagnostic only"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for field in ("dataset", "parent", "contexts", "tasks", "raw", "output"):
        parser.add_argument(field)
    args = parser.parse_args()
    output = Path(args.output)
    require(not output.exists(), "use a fresh audit output")
    report = verify(args.dataset, args.parent, args.contexts, args.tasks, args.raw)
    output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report, allow_nan=False))
