from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


def load_builder(path: Path):
    spec = importlib.util.spec_from_file_location("omml_builder", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Cannot load OMML builder")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> None:
    root = Path(__file__).resolve().parent
    formulas = json.loads((root / "lqi_formula_specs.json").read_text(encoding="utf-8"))
    builder = load_builder(Path(r"C:\Users\David_X\.codex\skills\word-native-equations\scripts\build_omml_docx.py"))
    out_dir = root / "formula_validation"
    out_dir.mkdir(exist_ok=True)
    audits = {}
    for name, formula in formulas.items():
        out = out_dir / f"{name}.docx"
        builder.write_docx(formula, out)
        audits[name] = builder.audit_docx(out)
    print(json.dumps(audits, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
