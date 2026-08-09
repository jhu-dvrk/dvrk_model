#!/usr/bin/env python3
"""Verify PSM instrument visual size and link-reference placement.

The instrument visual meshes are expected to be exported in the link frame
after the visual origin transform is applied. This check therefore uses an
ordinary transformed XYZ bounding box without a mesh geometry dependency.

The envelope is orientation-independent: one dimension may be up to 7 cm and
the other two dimensions may be up to 1 cm each. The link origin must also be
inside the combined visual bounds for each checked link.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml


MAX_DIMENSIONS = (0.07, 0.01, 0.01)
SI_ROLL_MAX_DIMENSIONS = (0.12, 0.02, 0.02)
ORIGIN_TOLERANCE = 1.0e-6
TARGET_LINKS = (
    "wrist_pitch_link",
    "wrist_yaw_link",
    "jaw_link",
    "jaw_1_link",
    "jaw_2_link",
)

Vector = Tuple[float, float, float]
Matrix = Tuple[Tuple[float, float, float], ...]


@dataclass
class Bounds:
    minimum: List[float] = field(default_factory=lambda: [math.inf] * 3)
    maximum: List[float] = field(default_factory=lambda: [-math.inf] * 3)

    def add(self, point: Vector) -> None:
        for index, value in enumerate(point):
            self.minimum[index] = min(self.minimum[index], value)
            self.maximum[index] = max(self.maximum[index], value)

    @property
    def dimensions(self) -> Tuple[float, float, float]:
        return tuple(
            maximum - minimum
            for minimum, maximum in zip(self.minimum, self.maximum)
        )

    def contains_origin(self, tolerance: float) -> bool:
        return all(
            minimum - tolerance <= 0.0 <= maximum + tolerance
            for minimum, maximum in zip(self.minimum, self.maximum)
        )


@dataclass
class InstrumentResult:
    instrument: str
    name: str
    errors: List[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.errors


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def load_instruments(root: Path) -> Dict[str, Dict[str, str]]:
    path = root / "urdf" / "common" / "instruments" / "instruments.yaml"
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return {str(key): value for key, value in data["instruments"].items()}


def psm_xacro_path(root: Path, generation: str) -> Path:
    return root / "urdf" / generation / "PSM1.urdf.xacro"


def run_xacro_expand(
    xacro_file: Path, instrument: str, is_virtual: bool = False
) -> str:
    command = ["xacro", str(xacro_file), f"instrument:={instrument}"]
    if is_virtual:
        command.append("is_virtual:=true")
    try:
        completed = subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            "xacro command not found. Source ROS and install xacro."
        ) from exc
    except subprocess.CalledProcessError as exc:
        detail = exc.stderr.strip() or exc.stdout.strip()
        raise RuntimeError(f"Failed to expand {xacro_file.name}: {detail}") from exc
    return completed.stdout


def parse_vector(value: Optional[str], default: Vector = (0.0, 0.0, 0.0)) -> Vector:
    if not value:
        return default
    values = tuple(float(part) for part in value.split())
    if len(values) != 3:
        raise ValueError(f"expected three values, found {value!r}")
    return values  # type: ignore[return-value]


def rotation_matrix(rpy: Vector) -> Matrix:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def transform_point(
    point: Vector, scale: Vector, rotation: Matrix, translation: Vector
) -> Vector:
    scaled = tuple(point[index] * scale[index] for index in range(3))
    rotated = tuple(
        sum(rotation[row][column] * scaled[column] for column in range(3))
        for row in range(3)
    )
    return tuple(rotated[index] + translation[index] for index in range(3))  # type: ignore[return-value]


def mesh_vertices(path: Path) -> Iterable[Vector]:
    if path.suffix.lower() != ".obj":
        raise ValueError(f"unsupported visual mesh format {path.suffix!r}: {path}")
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line_number, line in enumerate(stream, start=1):
            fields = line.split()
            if not fields or fields[0] != "v":
                continue
            if len(fields) < 4:
                raise ValueError(f"invalid OBJ vertex at {path}:{line_number}")
            yield (float(fields[1]), float(fields[2]), float(fields[3]))


def transformed_mesh_bounds(
    root: Path, mesh: ET.Element, visual: ET.Element
) -> Bounds:
    filename = mesh.get("filename", "")
    package_prefix = "package://dvrk_model/"
    if not filename.startswith(package_prefix):
        raise ValueError(f"unsupported mesh filename {filename!r}")
    mesh_path = root / filename[len(package_prefix):]
    if not mesh_path.is_file():
        raise ValueError(f"missing mesh {mesh_path}")

    scale = parse_vector(mesh.get("scale"), (1.0, 1.0, 1.0))
    origin = visual.find("origin")
    translation = parse_vector(origin.get("xyz") if origin is not None else None)
    rpy = parse_vector(origin.get("rpy") if origin is not None else None)
    rotation = rotation_matrix(rpy)

    vertices = list(mesh_vertices(mesh_path))
    if not vertices:
        raise ValueError(f"mesh has no OBJ vertices: {mesh_path}")

    local = Bounds()
    for vertex in vertices:
        local.add(vertex)

    # Transforming the eight corners is exact for an affine transform.
    result = Bounds()
    for x in (local.minimum[0], local.maximum[0]):
        for y in (local.minimum[1], local.maximum[1]):
            for z in (local.minimum[2], local.maximum[2]):
                result.add(transform_point((x, y, z), scale, rotation, translation))
    return result


def dimensions_fit_envelope(dimensions: Sequence[float]) -> bool:
    return all(
        actual <= expected + ORIGIN_TOLERANCE
        for actual, expected in zip(sorted(dimensions, reverse=True), MAX_DIMENSIONS)
    )


def format_dimensions(dimensions: Sequence[float]) -> str:
    return " x ".join(f"{value * 100.0:.2f} cm" for value in dimensions)


def combined_visual_bounds(
    root: Path, link_name: str, link: ET.Element
) -> Tuple[Optional[Bounds], List[str]]:
    bounds = Bounds()
    meshes_found = 0
    errors: List[str] = []
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            continue
        try:
            visual_bounds = transformed_mesh_bounds(root, mesh, visual)
        except (OSError, ValueError) as exc:
            errors.append(f"{link_name}: {exc}")
            continue
        meshes_found += 1
        for index in range(3):
            bounds.minimum[index] = min(
                bounds.minimum[index], visual_bounds.minimum[index]
            )
            bounds.maximum[index] = max(
                bounds.maximum[index], visual_bounds.maximum[index]
            )
    return (bounds if meshes_found else None), errors


def check_visuals(
    root: Path, instrument: str, config: Mapping[str, str], xml_text: str
) -> InstrumentResult:
    result = InstrumentResult(instrument, str(config.get("name", "<unnamed>")))
    urdf = ET.fromstring(xml_text)
    links = {
        link.get("name", ""): link
        for link in urdf.findall("link")
        if link.get("name")
    }

    for suffix in TARGET_LINKS:
        link_name = f"PSM1_{suffix}"
        link = links.get(link_name)
        if link is None:
            # Cautery tools intentionally have one fixed jaw_link and no
            # jaw_1_link/jaw_2_link pair.
            if suffix in ("jaw_1_link", "jaw_2_link"):
                continue
            result.errors.append(f"missing visual link {link_name}")
            continue

        bounds, errors = combined_visual_bounds(root, link_name, link)
        result.errors.extend(errors)

        # Some instruments intentionally use an empty jaw_link frame.
        if bounds is None:
            continue

        dimensions = bounds.dimensions
        if not dimensions_fit_envelope(dimensions):
            result.errors.append(
                f"{link_name}: visual bounds {format_dimensions(dimensions)} "
                f"exceed sorted envelope 7.00 cm x 1.00 cm x 1.00 cm"
            )
        if suffix in ("wrist_pitch_link", "wrist_yaw_link") and not bounds.contains_origin(
            ORIGIN_TOLERANCE
        ):
            result.errors.append(
                f"{link_name}: visual bounds do not contain the link origin; "
                f"bounds are {format_dimensions(dimensions)}"
            )

    # Si roll meshes are OBJ assets and can be oversized by a 10x scale typo,
    # which makes downstream wrist/jaw geometry look hidden inside the shaft.
    if str(config.get("housing", "")) == "Si":
        roll_link_name = "PSM1_roll_link"
        roll_link = links.get(roll_link_name)
        if roll_link is None:
            result.errors.append(f"missing visual link {roll_link_name}")
        else:
            roll_bounds, roll_errors = combined_visual_bounds(root, roll_link_name, roll_link)
            result.errors.extend(roll_errors)
            if roll_bounds is not None:
                roll_dimensions = roll_bounds.dimensions
                if not dimensions_fit_envelope(roll_dimensions):
                    if not all(
                        actual <= expected + ORIGIN_TOLERANCE
                        for actual, expected in zip(
                            sorted(roll_dimensions, reverse=True), SI_ROLL_MAX_DIMENSIONS
                        )
                    ):
                        result.errors.append(
                            f"{roll_link_name}: visual bounds {format_dimensions(roll_dimensions)} "
                            f"look oversized for Si roll; expected <= 12.00 cm x 2.00 cm x 2.00 cm. "
                            "This often hides the wrist inside the shaft."
                        )
    return result


def print_result(result: InstrumentResult) -> None:
    status = "PASS" if result.passed else "FAIL"
    print(f"[{status}] instrument {result.instrument}: {result.name}")
    for error in result.errors:
        print(f"  - {error}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instrument", help="single instrument model ID")
    parser.add_argument("--all", action="store_true", help="check all instruments")
    parser.add_argument(
        "--generation",
        choices=("Classic", "Si", "Virtual"),
        help="generation to check; defaults from the instrument housing",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.instrument and not args.all:
        parser.error("specify --instrument CODE or --all")

    root = repo_root()
    instruments = load_instruments(root)
    selected = [args.instrument] if args.instrument else sorted(instruments)
    unknown = sorted(set(selected) - set(instruments))
    if unknown:
        parser.error("unknown instrument(s): " + ", ".join(unknown))

    results: List[InstrumentResult] = []
    for instrument in selected:
        config = instruments[instrument]
        generation = args.generation
        if generation is None:
            generation = "Classic" if config["housing"] == "Classic" else "Si"
        xacro_file = psm_xacro_path(root, generation)
        try:
            xml_text = run_xacro_expand(
                xacro_file, instrument, is_virtual=generation == "Virtual"
            )
            result = check_visuals(root, instrument, config, xml_text)
        except (OSError, RuntimeError, ValueError, ET.ParseError) as exc:
            result = InstrumentResult(
                instrument,
                str(config.get("name", "<unnamed>")),
                [str(exc)],
            )
        results.append(result)
        print_result(result)

    failures = sum(not result.passed for result in results)
    print(f"\nChecked {len(results)} instrument(s). Failures: {failures}.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
