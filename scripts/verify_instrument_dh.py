#!/usr/bin/env python3
"""Check dvrk_model instrument mappings against the dVRK tool JSON files.

The tool JSON files are installed by the ``dvrk_config`` ROS package.  The
package can be located through ament or ``ros2 pkg prefix``; use
``--dvrk-tool-dir`` when checking an uninstalled source tree.

Examples:
    python3 scripts/verify_instrument_dh.py
    python3 scripts/verify_instrument_dh.py -v
    python3 scripts/verify_instrument_dh.py --instrument 420006 \
        --dvrk-tool-dir /path/to/dvrk_config/share/dvrk_config/tool
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml


PACKAGE_NAME = "dvrk_config"
FLOAT_TOLERANCE = 1.0e-7
EXPECTED_ALPHA = -1.5708
EXPECTED_ROLL_D = {"Classic": 0.4162, "Si": 0.4670}
EXPECTED_WRIST_A = {
    "0091": 0.0091,
    "0093": 0.0093,
    "0100": 0.0100,
    "0107": 0.0107,
    "0112": 0.0112,
}


@dataclass
class InstrumentResult:
    instrument: str
    name: str
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.errors


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def load_instruments(root: Path) -> Dict[str, Dict[str, Any]]:
    path = root / "urdf" / "common" / "instruments" / "instruments.yaml"
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    instruments = data.get("instruments") if isinstance(data, dict) else None
    if not isinstance(instruments, dict):
        raise ValueError(f"Invalid instrument table: {path}")
    return {str(key): value for key, value in instruments.items()}


def clean_json(text: str) -> str:
    """Convert dVRK's comment-bearing JSON files to strict JSON."""
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//.*", "", text)
    return re.sub(r",\s*([}\]])", r"\1", text)


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.loads(clean_json(stream.read()))
    if not isinstance(value, dict):
        raise ValueError("top-level JSON value is not an object")
    return value


def package_share_directory(package_name: str) -> Optional[Path]:
    """Locate a ROS package without requiring a Python ROS environment."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name))
    except (ImportError, LookupError, RuntimeError):
        pass

    ros2 = shutil.which("ros2")
    if ros2 is None:
        return None
    try:
        prefix = subprocess.run(
            [ros2, "pkg", "prefix", package_name],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None

    # ROS 2 installs the dvrk_config share package at share/dvrk_config.
    candidate = Path(prefix) / "share" / package_name
    return candidate if candidate.is_dir() else None


def resolve_tool_dir(explicit: Optional[Path], package_name: str) -> Path:
    if explicit is not None:
        path = explicit.expanduser().resolve()
        if not path.is_dir():
            raise ValueError(f"dvrk tool directory does not exist: {path}")
        return path

    share = package_share_directory(package_name)
    if share is None:
        raise ValueError(
            f"Could not locate ROS package {package_name!r}; source ROS or use "
            "--dvrk-tool-dir PATH"
        )
    tool_dir = share / "tool"
    if not tool_dir.is_dir():
        raise ValueError(f"ROS package has no tool directory: {tool_dir}")
    return tool_dir


def nearly_equal(actual: Any, expected: float) -> bool:
    try:
        return abs(float(actual) - expected) <= FLOAT_TOLERANCE
    except (TypeError, ValueError):
        return False


def get_joint_map(data: Mapping[str, Any]) -> Tuple[Optional[Dict[str, Dict[str, Any]]], Optional[str]]:
    dh = data.get("DH")
    if not isinstance(dh, dict):
        return None, "missing DH object"
    if dh.get("convention") != "modified":
        return None, f"DH convention is {dh.get('convention')!r}, expected 'modified'"
    joints = dh.get("joints")
    if not isinstance(joints, list):
        return None, "DH joints is not a list"
    result: Dict[str, Dict[str, Any]] = {}
    for joint in joints:
        if not isinstance(joint, dict) or not isinstance(joint.get("name"), str):
            return None, "DH joints contains an invalid entry"
        name = joint["name"]
        if name in result:
            return None, f"duplicate DH joint {name!r}"
        result[name] = joint
    return result, None


def check_number(
    errors: List[str], joint: Mapping[str, Any], field_name: str, expected: float
) -> None:
    actual = joint.get(field_name)
    if not nearly_equal(actual, expected):
        errors.append(
            f"DH {joint.get('name', '<unknown>')}.{field_name} expected "
            f"{expected:g}, found {actual!r}"
        )


def check_dh(
    errors: List[str], config: Mapping[str, Any], data: Mapping[str, Any]
) -> None:
    joints, error = get_joint_map(data)
    if error:
        errors.append(error)
        return
    assert joints is not None

    required = ("roll", "wrist_pitch", "wrist_yaw")
    for name in required:
        if name not in joints:
            errors.append(f"missing standard DH joint {name!r}")
    if any(name not in joints for name in required):
        return

    for name in required:
        if joints[name].get("type") != "revolute":
            errors.append(f"DH joint {name!r} is not revolute")

    housing = str(config.get("housing"))
    roll = str(config.get("roll"))
    expected_d = EXPECTED_ROLL_D.get(housing)
    if expected_d is None:
        errors.append(f"unsupported YAML housing {housing!r}")
    elif roll != f"{expected_d * 10000:.0f}":
        errors.append(f"YAML roll {roll!r} does not map to {expected_d:g} m")
    else:
        check_number(errors, joints["roll"], "D", expected_d)

    check_number(errors, joints["roll"], "A", 0.0)
    check_number(errors, joints["roll"], "alpha", 0.0)
    check_number(errors, joints["wrist_pitch"], "A", 0.0)
    check_number(errors, joints["wrist_pitch"], "alpha", EXPECTED_ALPHA)
    check_number(errors, joints["wrist_yaw"], "alpha", EXPECTED_ALPHA)

    wrist_code = str(config.get("wrist_yaw"))
    expected_a = EXPECTED_WRIST_A.get(wrist_code)
    if expected_a is None:
        errors.append(f"unsupported YAML wrist_yaw code {wrist_code!r}")
    else:
        check_number(errors, joints["wrist_yaw"], "A", expected_a)


def xacro_path(root: Path, part: str, value: str) -> Path:
    if part == "housing":
        return root / "urdf" / "common" / "instruments" / "housing" / f"housing_{value}.urdf.xacro"
    return root / "urdf" / "common" / "instruments" / part / f"{part}_{value}.urdf.xacro"


def check_xacro_dh(root: Path, config: Mapping[str, Any], result: InstrumentResult) -> None:
    """Check the numeric origins used by dvrk_model's stage Xacros."""
    housing = str(config.get("housing"))
    wrist_code = str(config.get("wrist_yaw"))
    expected_d = EXPECTED_ROLL_D.get(housing)
    expected_a = EXPECTED_WRIST_A.get(wrist_code)
    if expected_d is None or expected_a is None:
        return

    checks = [
        ("roll", "roll", "z", expected_d),
        ("wrist_pitch", "wrist_pitch", "x", 0.0),
        ("wrist_yaw", "wrist_yaw", "x", expected_a),
    ]
    for part, joint_name, coordinate, expected in checks:
        path = xacro_path(root, part, str(config.get(part)))
        if not path.is_file():
            continue
        try:
            tree = ET.parse(path)
        except ET.ParseError:
            continue
        joints = [joint for joint in tree.findall(".//joint") if joint.get("name") == joint_name]
        if not joints:
            result.errors.append(f"missing {joint_name!r} joint in {path.name}")
            continue
        origin = joints[0].find("origin")
        actual = None
        if origin is not None:
            xyz = origin.get("xyz", "").split()
            coordinate_index = {"x": 0, "y": 1, "z": 2}[coordinate]
            if len(xyz) == 3:
                actual = xyz[coordinate_index]
        if actual is None:
            result.errors.append(f"missing origin {coordinate!r} for {joint_name} in {path.name}")
        elif not nearly_equal(actual, expected):
            result.errors.append(
                f"{path.name} {joint_name} origin {coordinate} expected {expected:g}, "
                f"found {actual!r}"
            )


def mesh_paths(root: Path, xacro: Path) -> Iterable[Path]:
    tree = ET.parse(xacro)
    for mesh in tree.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        prefix = "package://dvrk_model/"
        if filename.startswith(prefix):
            yield root / filename[len(prefix) :]


def check_urdf_assets(
    root: Path, config: Mapping[str, Any], result: InstrumentResult
) -> None:
    values = {
        "housing": str(config.get("housing")),
        "roll": str(config.get("roll")),
        "wrist_pitch": str(config.get("wrist_pitch")),
        "wrist_yaw": str(config.get("wrist_yaw")),
    }
    for part, value in values.items():
        path = xacro_path(root, part, value)
        if not path.is_file():
            result.errors.append(f"missing {part} Xacro: {path}")
            continue
        try:
            for mesh in mesh_paths(root, path):
                if not mesh.is_file():
                    result.errors.append(f"missing mesh referenced by {path.name}: {mesh}")
        except ET.ParseError as exc:
            result.errors.append(f"invalid XML in {path}: {exc}")

    check_xacro_dh(root, config, result)

    tip_code = str(config.get("tip"))
    tip = xacro_path(root, "tip", tip_code)
    if not tip.is_file():
        placeholder = root / "urdf" / "common" / "instruments" / "tip" / "tip_placeholder.urdf.xacro"
        if placeholder.is_file():
            result.warnings.append(f"tip {tip_code} uses tip_placeholder.urdf.xacro")
        else:
            result.errors.append(f"missing tip Xacro and placeholder for tip {tip_code}")
        return
    try:
        for mesh in mesh_paths(root, tip):
            if not mesh.is_file():
                result.errors.append(f"missing tip mesh referenced by {tip.name}: {mesh}")
    except ET.ParseError as exc:
        result.errors.append(f"invalid XML in {tip}: {exc}")


def check_instrument(
    root: Path,
    tool_dir: Path,
    instrument: str,
    config: Mapping[str, Any],
    index: Mapping[str, Path],
) -> InstrumentResult:
    result = InstrumentResult(instrument, str(config.get("name", "<unnamed>")))
    filename = index.get(instrument)
    if filename is None:
        result.errors.append("instrument is missing from dvrk_config/tool/index.json")
    else:
        path = tool_dir / filename
        if not path.is_file():
            result.errors.append(f"missing tool JSON: {path}")
        else:
            try:
                check_dh(result.errors, config, load_json(path))
            except (OSError, json.JSONDecodeError, ValueError) as exc:
                result.errors.append(f"could not parse {path.name}: {exc}")

    check_urdf_assets(root, config, result)
    return result


def load_index(tool_dir: Path) -> Tuple[Dict[str, Path], List[str]]:
    path = tool_dir / "index.json"
    data = load_json(path)
    records = data.get("instruments")
    if not isinstance(records, list):
        raise ValueError(f"{path} has no instruments list")
    index: Dict[str, Path] = {}
    errors: List[str] = []
    for record in records:
        if not isinstance(record, dict):
            errors.append("index contains a non-object instrument record")
            continue
        model = str(record.get("model", ""))
        filename = record.get("file")
        if not model or not isinstance(filename, str):
            errors.append("index contains a record without model/file")
            continue
        # Some S instruments have generation-specific geared variants with
        # the same model ID. Prefer the ordinary file for the generic DH
        # check; both variants are checked for existence below.
        candidate = Path(filename)
        if model not in index or "_GEARED" not in candidate.stem:
            index[model] = candidate
        if Path(filename).name != filename or not filename.endswith(".json"):
            errors.append(f"invalid tool filename for {model}: {filename}")
        if not (tool_dir / candidate).is_file():
            errors.append(f"missing tool JSON listed for {model}: {candidate}")
    return index, errors


def print_result(result: InstrumentResult, verbose: bool) -> None:
    if result.passed and not result.warnings and not verbose:
        return
    if result.errors:
        status = "FAIL"
    elif result.warnings:
        status = "WARN"
    else:
        status = "PASS"
    print(f"[{status}] instrument {result.instrument}: {result.name}")
    if result.errors:
        for error in result.errors:
            print(f"  - {error}")
    if result.warnings:
        for warning in result.warnings:
            print(f"  - warning: {warning}")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--instrument", help="check one instrument model ID")
    result.add_argument("--dvrk-tool-dir", type=Path, help="path to dvrk_config/tool")
    result.add_argument(
        "--dvrk-package",
        default=PACKAGE_NAME,
        help=f"ROS package used to locate tool JSON files (default: {PACKAGE_NAME})",
    )
    result.add_argument("-v", "--verbose", action="store_true", help="also print passing instruments")
    return result


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parser().parse_args(argv)
    root = repo_root()
    try:
        instruments = load_instruments(root)
        tool_dir = resolve_tool_dir(args.dvrk_tool_dir, args.dvrk_package)
        index, index_errors = load_index(tool_dir)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    selected = [args.instrument] if args.instrument else sorted(instruments)
    unknown = [model for model in selected if model not in instruments]
    if unknown:
        print(f"ERROR: instrument(s) not in instruments.yaml: {', '.join(unknown)}", file=sys.stderr)
        return 2

    for error in index_errors:
        print(f"[FAIL] index.json: {error}")

    results = [
        check_instrument(root, tool_dir, model, instruments[model], index)
        for model in selected
    ]
    for result in results:
        print_result(result, args.verbose)

    if not args.instrument:
        extras = sorted(set(index) - set(instruments))
        if extras:
            print(f"[WARN] instruments in dvrk_config but not instruments.yaml: {', '.join(extras)}")

    failures = len(index_errors) + sum(not result.passed for result in results)
    warnings = sum(bool(result.warnings) for result in results)
    print(f"\nChecked {len(results)} instrument(s). Failures: {failures}. Warnings: {warnings}.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
