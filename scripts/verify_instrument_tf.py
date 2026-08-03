#!/usr/bin/env python3
"""Verify instrument TF/link chain and naming for dVRK PSM URDF models.

This script expands a PSM xacro for one instrument at a time and validates:
1) Naming scheme for instrument links
2) Parent/child link assignments for stage joints
3) Connectivity from adaptor link through tool-tip and jaw links

Usage examples:
    python3 scripts/verify_instrument_tf.py --instrument 420006
    python3 scripts/verify_instrument_tf.py --all
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Set, Tuple

import yaml


EXPECTED_STAGE_JOINTS = {
    "housing_fixed": ("adaptor_link", "housing_link"),
    "roll": ("adaptor_link", "roll_link"),
    "wrist_pitch": ("roll_link", "wrist_pitch_link"),
    "wrist_yaw": ("wrist_pitch_link", "wrist_yaw_link"),
    "jaw": ("wrist_yaw_link", "jaw_link"),
    "jaw_1": ("wrist_yaw_link", "jaw_1_link"),
    "jaw_2": ("wrist_yaw_link", "jaw_2_link"),
    "tool_tip": ("wrist_yaw_link", "tool_tip_link"),
}

REQUIRED_STAGE_LINK_SUFFIXES = {
    "adaptor_link",
    "housing_link",
    "roll_link",
    "wrist_pitch_link",
    "wrist_yaw_link",
    "jaw_link",
    "jaw_1_link",
    "jaw_2_link",
    "tool_tip_link",
}


@dataclass
class CheckResult:
    instrument: str
    generation: str
    passed: bool
    errors: List[str]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def load_instrument_table(root: Path) -> Dict[str, Dict[str, str]]:
    yaml_path = root / "urdf" / "common" / "instruments" / "instruments.yaml"
    with yaml_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data["instruments"]


def infer_generation(instrument_cfg: Dict[str, str]) -> str:
    housing = str(instrument_cfg["housing"])
    if housing == "Classic":
        return "Classic"
    if housing == "Si":
        return "Si"
    raise ValueError(f"Unsupported housing/generation mapping: {housing}")


def psm_xacro_path(root: Path, generation: str) -> Path:
    return root / "urdf" / generation / "PSM1.urdf.xacro"


def run_xacro_expand(xacro_file: Path, instrument: str) -> str:
    cmd = ["xacro", str(xacro_file), f"instrument:={instrument}"]
    try:
        out = subprocess.run(
            cmd,
            check=True,
            capture_output=True,
            text=True,
        )
        return out.stdout
    except FileNotFoundError as exc:
        raise RuntimeError(
            "xacro command not found. Source ROS environment and install xacro."
        ) from exc
    except subprocess.CalledProcessError:
        alt_cmd = ["ros2", "run", "xacro", "xacro", str(xacro_file), f"instrument:={instrument}"]
        try:
            out = subprocess.run(
                alt_cmd,
                check=True,
                capture_output=True,
                text=True,
            )
            return out.stdout
        except Exception as exc:  # pylint: disable=broad-except
            raise RuntimeError(
                "Failed to expand xacro. Ensure ROS is sourced and dvrk_model is built."
            ) from exc


def parse_urdf(xml_text: str) -> Tuple[Set[str], Dict[str, str], Dict[str, Tuple[str, str]]]:
    root = ET.fromstring(xml_text)

    links: Set[str] = set()
    child_to_parent: Dict[str, str] = {}
    joint_links: Dict[str, Tuple[str, str]] = {}

    for link in root.findall("link"):
        name = link.attrib.get("name", "")
        if name:
            links.add(name)

    for joint in root.findall("joint"):
        jname = joint.attrib.get("name", "")
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        if parent_elem is None or child_elem is None:
            continue
        parent = parent_elem.attrib.get("link", "")
        child = child_elem.attrib.get("link", "")
        if parent and child:
            child_to_parent[child] = parent
            joint_links[jname] = (parent, child)

    return links, child_to_parent, joint_links


def verify_instrument(
    instrument: str,
    generation: str,
    xml_text: str,
    prefix: str = "PSM1_",
) -> CheckResult:
    errors: List[str] = []

    links, child_to_parent, joint_links = parse_urdf(xml_text)

    required_links = {f"{prefix}{suffix}" for suffix in REQUIRED_STAGE_LINK_SUFFIXES}

    missing_links = sorted(required_links - links)
    if missing_links:
        errors.append("Missing required instrument links: " + ", ".join(missing_links))

    # Naming scheme: all required stage links must use the exact PSM prefix.
    for suffix in sorted(REQUIRED_STAGE_LINK_SUFFIXES):
        expected_name = f"{prefix}{suffix}"
        if expected_name not in links:
            continue
        if not expected_name.startswith(prefix):
            errors.append(f"Naming mismatch for link {expected_name}")

    # Parent/child checks for stage joints.
    for joint_name, (expected_parent_suffix, expected_child_suffix) in EXPECTED_STAGE_JOINTS.items():
        joint_candidates = [joint_name, f"{prefix}{joint_name}"]
        selected_joint_name = ""
        for candidate in joint_candidates:
            if candidate in joint_links:
                selected_joint_name = candidate
                break

        if not selected_joint_name:
            errors.append(f"Missing joint: {joint_name}")
            continue
        parent, child = joint_links[selected_joint_name]
        expected_parent = f"{prefix}{expected_parent_suffix}"
        expected_child = f"{prefix}{expected_child_suffix}"
        if parent != expected_parent or child != expected_child:
            errors.append(
                f"Joint {selected_joint_name} expected {expected_parent} -> {expected_child}, "
                f"found {parent} -> {child}"
            )

    # Connectivity checks via ancestor traversal.
    base = f"{prefix}adaptor_link"
    must_reach = [
        f"{prefix}housing_link",
        f"{prefix}roll_link",
        f"{prefix}wrist_pitch_link",
        f"{prefix}wrist_yaw_link",
        f"{prefix}jaw_link",
        f"{prefix}jaw_1_link",
        f"{prefix}jaw_2_link",
        f"{prefix}tool_tip_link",
    ]

    for target in must_reach:
        cur = target
        visited: Set[str] = set()
        reachable = False
        while cur and cur not in visited:
            if cur == base:
                reachable = True
                break
            visited.add(cur)
            cur = child_to_parent.get(cur, "")
        if not reachable:
            errors.append(f"Connectivity failure: {target} does not trace back to {base}")

    return CheckResult(
        instrument=instrument,
        generation=generation,
        passed=not errors,
        errors=errors,
    )


def print_result(result: CheckResult, verbose: bool) -> None:
    status = "PASS" if result.passed else "FAIL"
    print(f"[{status}] instrument {result.instrument} ({result.generation})")
    if (verbose or not result.passed) and result.errors:
        for err in result.errors:
            print(f"  - {err}")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Verify dVRK instrument TF naming, parent links, and connectivity."
    )
    parser.add_argument(
        "--instrument",
        type=str,
        help="Single 6-digit instrument code to verify, e.g. 420006",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Verify all instruments listed in urdf/common/instruments/instruments.yaml",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print details for all checks, not only failures",
    )
    return parser


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()

    if not args.instrument and not args.all:
        parser.error("Specify --instrument CODE or --all")

    root = repo_root()
    table = load_instrument_table(root)

    instruments: List[str]
    if args.instrument:
        instruments = [args.instrument]
    else:
        instruments = sorted(table.keys())

    results: List[CheckResult] = []

    for instrument in instruments:
        cfg = table.get(instrument)
        if cfg is None:
            results.append(
                CheckResult(
                    instrument=instrument,
                    generation="unknown",
                    passed=False,
                    errors=[f"Instrument {instrument} not found in instruments.yaml"],
                )
            )
            continue

        generation = infer_generation(cfg)
        xacro_file = psm_xacro_path(root, generation)

        if not xacro_file.exists():
            results.append(
                CheckResult(
                    instrument=instrument,
                    generation=generation,
                    passed=False,
                    errors=[f"Missing xacro file: {xacro_file}"],
                )
            )
            continue

        try:
            urdf_xml = run_xacro_expand(xacro_file, instrument)
            result = verify_instrument(instrument, generation, urdf_xml)
        except Exception as exc:  # pylint: disable=broad-except
            result = CheckResult(
                instrument=instrument,
                generation=generation,
                passed=False,
                errors=[str(exc)],
            )
        results.append(result)
        print_result(result, args.verbose)

    failures = [r for r in results if not r.passed]
    print()
    print(f"Verified {len(results)} instrument(s). Failures: {len(failures)}")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
