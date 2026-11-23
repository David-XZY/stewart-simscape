"""Copy complete frozen F tasks into a new, byte-verified transfer snapshot."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def snapshot(source, output, previous=()):
    source, output = Path(source).resolve(), Path(output).resolve()
    frozen = read(source / "source_manifest.json")
    expected = [task["taskId"] for task in frozen["tasks"]]
    if (len(expected) != 12 or len(set(expected)) != 12
            or any(not re.fullmatch(r"task_[0-9]+", task_id) for task_id in expected)):
        raise ValueError("Expected all twelve original test task identities")
    excluded = set()
    for path in previous:
        prior = read(Path(path) / "transfer_manifest.json")
        if prior["state"] != "complete" or not set(prior["taskIds"]).issubset(expected):
            raise ValueError("Invalid previous snapshot")
        excluded.update(prior["taskIds"])
    state = read(source / "pipeline_status.json")
    completed = state["completedTasks"]
    if len(set(completed)) != len(completed) or not set(completed).issubset(expected):
        raise ValueError("Invalid native completion list")
    selected = [task_id for task_id in expected if task_id in completed and task_id not in excluded]
    if not selected:
        raise ValueError("No newly completed tasks; no empty snapshot created")
    files = [Path("source_manifest.json"), Path("pipeline_status.json")]
    core_hashes = {}
    for task_id in selected:
        prefix = Path("tasks") / task_id
        status_path = source / prefix / "task_status.json"
        core_hashes[prefix / "task_status.json"] = sha(status_path)
        status = read(status_path)
        methods = status["methods"]
        if (status["state"] != "complete" or status["taskId"] != task_id
                or [method["method"] for method in methods] != ["geometric-single", "nearest", "mlp"]
                or any(type(method["passed"]) is not bool for method in methods)):
            raise ValueError("Missing complete native three-method result")
        files.extend(prefix / name for name in ("manifest.json", "task_status.json", "warmstart_benchmark.json"))
        for index, method in enumerate(methods, 1):
            directory = prefix / task_id / f"method_{index:02d}"
            if index > 1:
                files.append(directory / "ranking.mat")
            if method["passed"]:
                files.append(directory / "best_result.mat")
    output.mkdir(parents=True, exist_ok=False)
    entries = []
    for relative in files:
        origin = source / relative
        target_relative = Path(*relative.parts[1:]) if relative.parts[0] == "tasks" else relative
        destination = output / target_relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        before = sha(origin)
        if relative.name == "pipeline_status.json":
            # This supervisor file may advance while numerical tasks keep running.
            destination.write_text(json.dumps(state, indent=2), encoding="utf-8")
        else:
            shutil.copyfile(origin, destination)
            if sha(destination) != before or sha(origin) != before:
                raise ValueError("Source changed during transfer; snapshot remains incomplete")
        entries.append({"path": target_relative.as_posix(), "sha256": sha(destination),
                        "bytes": destination.stat().st_size})
    for relative, expected_hash in core_hashes.items():
        if sha(source / relative) != expected_hash:
            raise ValueError("Native task completion changed during transfer")
    manifest = {"state": "complete", "captured": datetime.now(timezone.utc).isoformat(),
                "source": str(source), "taskIds": selected, "expectedTaskIds": expected,
                "previousTaskIds": sorted(excluded), "files": entries,
                "scope": "Completed tasks only, all methods retained regardless of outcome; local MATLAB must audit candidates and references again"}
    (output / "transfer_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--exclude-snapshot", type=Path, action="append", default=[])
    args = parser.parse_args()
    result = snapshot(args.source, args.output, args.exclude_snapshot)
    print(json.dumps({"state": result["state"], "taskIds": result["taskIds"],
                      "fileCount": len(result["files"])}))


if __name__ == "__main__":
    main()
