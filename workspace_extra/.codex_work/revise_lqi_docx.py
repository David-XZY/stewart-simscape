from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import sys
import zipfile
from pathlib import Path

from lxml import etree


W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
M_NS = "http://schemas.openxmlformats.org/officeDocument/2006/math"
W14_NS = "http://schemas.microsoft.com/office/word/2010/wordml"
NS = {"w": W_NS, "m": M_NS, "w14": W14_NS}


def qn(prefix: str, local: str) -> str:
    return f"{{{NS[prefix]}}}{local}"


def load_builder(path: Path):
    spec = importlib.util.spec_from_file_location("omml_builder", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Cannot load the native OfficeMath builder")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def p_text(paragraph) -> str:
    return "".join(paragraph.xpath(".//w:t/text()", namespaces=NS))


def replace_text(paragraph, text: str) -> None:
    ppr = paragraph.find("w:pPr", NS)
    rpr = paragraph.find("w:r/w:rPr", NS)
    for child in list(paragraph):
        if child is not ppr:
            paragraph.remove(child)
    run = etree.Element(qn("w", "r"))
    if rpr is not None:
        run.append(copy.deepcopy(rpr))
    node = etree.SubElement(run, qn("w", "t"))
    node.text = text
    paragraph.append(run)


def set_label(paragraph, label: str) -> None:
    runs = paragraph.findall("w:r", NS)
    if not runs:
        raise RuntimeError("Formula paragraph has no number run")
    label_run = runs[-1]
    texts = label_run.findall("w:t", NS)
    if not texts:
        text = etree.SubElement(label_run, qn("w", "t"))
    else:
        text = texts[-1]
        for extra in texts[:-1]:
            label_run.remove(extra)
    text.text = label


def replace_formula(paragraph, template, label: str, items, builder) -> None:
    ppr = paragraph.find("w:pPr", NS)
    tab_run = template.findall("w:r", NS)[0]
    label_run = template.findall("w:r", NS)[-1]
    for child in list(paragraph):
        if child is not ppr:
            paragraph.remove(child)
    paragraph.append(copy.deepcopy(tab_run))
    math = builder.m("oMath")
    for item in items:
        math.append(builder.make_node(item))
    paragraph.append(math)
    label_copy = copy.deepcopy(label_run)
    label_texts = label_copy.findall("w:t", NS)
    if not label_texts:
        label_texts = [etree.SubElement(label_copy, qn("w", "t"))]
    label_texts[-1].text = label
    for extra in label_texts[:-1]:
        label_copy.remove(extra)
    paragraph.append(label_copy)


def cloned_text(template, text: str):
    paragraph = copy.deepcopy(template)
    replace_text(paragraph, text)
    return paragraph


def cloned_formula(template, label: str, items, builder):
    paragraph = copy.deepcopy(template)
    replace_formula(paragraph, template, label, items, builder)
    return paragraph


def package_hashes(path: Path) -> dict[str, str]:
    with zipfile.ZipFile(path) as archive:
        return {info.filename: hashlib.sha256(archive.read(info.filename)).hexdigest() for info in archive.infolist()}


def assign_unique_paragraph_ids(body, new_nodes) -> None:
    para_id_attr = f"{{{W14_NS}}}paraId"
    text_id_attr = f"{{{W14_NS}}}textId"
    retained_ids = {
        paragraph.get(para_id_attr)
        for paragraph in body.findall("w:p", NS)
        if paragraph not in new_nodes and paragraph.get(para_id_attr)
    }
    value = 0x60000000
    for paragraph in new_nodes:
        while f"{value:08X}" in retained_ids:
            value += 1
        paragraph.set(para_id_attr, f"{value:08X}")
        paragraph.set(text_id_attr, f"{(value ^ 0x5A5A5A5A):08X}")
        retained_ids.add(f"{value:08X}")
        value += 1


def main() -> None:
    root = Path(__file__).resolve().parent
    src = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQR原理细化版.docx")
    out = Path(r"D:\NPU\并联机器人\控制\第四章 Stewart平台轨迹跟踪控制_计算力矩调度LQI修订稿_LQI闭环与龙伯格观测器修订版.docx")
    expected_hash = "68A105BCC7113A149320FB7BBF83FC5E22978A87316FD9A35F82DFC9C389BE77"
    if sha256(src) != expected_hash:
        raise RuntimeError("Retained document hash changed; fresh template inspection is required")

    formulas = json.loads((root / "lqi_formula_specs.json").read_text(encoding="utf-8"))
    builder = load_builder(Path(r"C:\Users\David_X\.codex\skills\word-native-equations\scripts\build_omml_docx.py"))
    before_parts = package_hashes(src)

    with zipfile.ZipFile(src) as archive:
        document_xml = archive.read("word/document.xml")
        parts = [(copy.copy(info), archive.read(info.filename)) for info in archive.infolist()]

    root_xml = etree.fromstring(document_xml)
    body = root_xml.find("w:body", NS)
    if body is None:
        raise RuntimeError("No document body")
    children = list(body)
    expected = {
        46: "4.2.3 从LQR到参考轨迹调度LQI的增广设计",
        47: "为消除局部LQR在常值扰动和静态模型偏差下可能出现的稳态误差",
        57: "4.2.4 计算力矩-调度LQI控制结构与实现",
    }
    for index, phrase in expected.items():
        if index >= len(children) or phrase not in p_text(children[index]):
            raise RuntimeError(f"Unexpected document structure at body child {index}")

    text_template = copy.deepcopy(children[47])
    formula_template = copy.deepcopy(children[54])
    antiwindup_original = copy.deepcopy(children[54])
    force_original = copy.deepcopy(children[56])
    eq_46 = copy.deepcopy(children[48])
    eq_47 = copy.deepcopy(children[49])
    eq_48 = copy.deepcopy(children[51])

    for child in reversed(children[47:57]):
        body.remove(child)

    new_nodes = [
        cloned_text(text_template, "局部误差模型式（4-4）已经把非线性平台在第k个参考工作点附近表示为输入为反馈修正力的线性系统。LQR仅根据当前误差状态给出比例型状态反馈，因而对恒定负载偏差或常值模型失配只能改善瞬态，未必消除稳态偏差。为引入内部模型作用，取位姿误差输出 y=Cδx，并定义积分状态 x_I=-∫y dt；于是增广状态和增广误差系统分别为"),
        eq_46,
        eq_47,
        cloned_text(text_template, "在增广系统上以 Q_a⪰0 和 R≻0 构造与式（4-5）相同的无限时域二次型指标，即得到 LQI。其反馈修正力由状态误差和累计位姿误差共同决定："),
        eq_48,
        cloned_formula(formula_template, "(4-9)", formulas["control"]["items"], builder),
        cloned_text(text_template, "将式（4-9）代入式（4-7），在全状态可得、执行器未饱和且暂不计抗饱和回灌时，增广闭环状态方程直接写为式（4-10）。其中 K_x,k 决定瞬时状态误差的衰减，K_I,k 利用累计位姿误差抵消常值扰动或静态模型偏差。理想积分器取 λ=0 时，若增广闭环稳定且常值扰动满足匹配条件，可实现零稳态位姿误差；实际控制器通常取 λ>0 以抑制积分累积，此时“零稳态误差”应理解为设计目标而非无条件严格结论。"),
        cloned_formula(formula_template, "(4-10)", formulas["state_closed"]["items"], builder),
        cloned_text(text_template, "当速度等状态不能直接测得时，LQI不能把未测量量简单置零，而应由局部模型和可测位姿误差进行状态重构。采用与本文符号一致的 Luenberger 观测器，令 δx_obs 为误差状态估计值，并令 e_o=δx_obs-δx，则观测器和估计误差动力学为"),
        cloned_formula(formula_template, "(4-11)", formulas["observer_error"]["items"], builder),
        cloned_text(text_template, "式（4-11）给出了观测器收敛需要证明或至少需要验证的对象：对冻结在第k个参考工作点的局部模型，必须选择 L_k 使 A_k+L_kC 为 Hurwitz 矩阵，即其全部特征值实部为负。等价地，要求 (A_k,C) 可检测；在该条件下可通过对偶 LQR 或极点配置求得 L_k。该检查说明估计误差在局部模型中指数衰减，而不等同于对真实非线性平台的全局观测器收敛证明。"),
        cloned_text(text_template, "把观测误差视为闭环的第三组状态，并令 z_k=[δx^T,x_I^T,e_o^T]^T，可将观测器型 LQI 的名义闭环写成式（4-12）。该矩阵为块上三角结构：增广状态反馈块和观测误差块均稳定时，冻结工作点的线性闭环稳定。"),
        cloned_formula(formula_template, "(4-12)", formulas["output_closed"]["items"], builder),
        cloned_text(text_template, "对参考轨迹调度情形，逐个采样点检查上述两个稳定性条件，是控制器与观测器的必要设计核查；但由于 A_k、B_k、K_k 和 L_k 随时间切换，逐点极点在左半平面并不能单独构成整条轨迹上的全局或统一指数稳定性证明。若论文需要作出该强结论，还需给出公共或参数依赖 Lyapunov 函数、调度变化率或切换驻留时间等附加论证。本文将结论限定为参考轨迹邻域内的局部设计与数值验证。"),
        cloned_text(text_template, "为防止执行器饱和、死区和速率限制使积分状态持续累积，实际实现保留积分泄漏，并利用实际执行力与原始命令力之差进行抗饱和回灌："),
        antiwindup_original,
        cloned_text(text_template, "计算力矩前馈与调度 LQI 的反馈修正力相加，形成后续 QP 在线校正层使用的名义支链力："),
        force_original,
    ]
    set_label(eq_46, "(4-6)")
    set_label(eq_47, "(4-7)")
    set_label(eq_48, "(4-8)")
    set_label(antiwindup_original, "(4-13)")
    set_label(force_original, "(4-14)")
    assign_unique_paragraph_ids(body, new_nodes)
    insertion_index = 47
    for node in new_nodes:
        body.insert(insertion_index, node)
        insertion_index += 1

    qp_found = False
    for paragraph in body.findall("w:p", NS):
        math_text = "".join(paragraph.xpath(".//m:t/text()", namespaces=NS))
        if "Fcmd" in math_text:
            set_label(paragraph, "(4-15)")
            qp_found = True
            break
    if not qp_found:
        raise RuntimeError("QP equation was not found")

    updated_document_xml = etree.tostring(root_xml, encoding="UTF-8", xml_declaration=True, standalone=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(out, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for info, payload in parts:
            archive.writestr(info, updated_document_xml if info.filename == "word/document.xml" else payload)

    if sha256(src) != expected_hash:
        raise RuntimeError("Retained document changed during revision")
    after_parts = package_hashes(out)
    changed_parts = sorted(name for name in set(before_parts) | set(after_parts) if before_parts.get(name) != after_parts.get(name))
    if changed_parts != ["word/document.xml"]:
        raise RuntimeError(f"Unexpected package changes: {changed_parts}")

    with zipfile.ZipFile(out) as archive:
        final_root = etree.fromstring(archive.read("word/document.xml"))
    labels = final_root.xpath(".//w:t[starts-with(text(), '(')]/text()", namespaces=NS)
    report = {
        "output": str(out),
        "source_sha256": expected_hash,
        "output_sha256": sha256(out),
        "changed_package_parts": changed_parts,
        "oMath_count": int(final_root.xpath("count(.//m:oMath)", namespaces=NS)),
        "equation_labels": [label for label in labels if label.startswith("(4-")],
        "new_equation_audits": {
            name: builder.audit_docx(root / "formula_validation" / f"{name}.docx")
            for name in formulas
        },
    }
    report_path = root / "lqi_docx_revision" / "revision_report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
