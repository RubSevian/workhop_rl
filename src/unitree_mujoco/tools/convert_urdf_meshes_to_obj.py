#!/usr/bin/env python3
"""Create lossless OBJ-backed derivatives of the canonical RARS01 URDFs.

The source URDF and STL files are read-only inputs.  Conversion deliberately
uses ``process=False`` and performs no repair, vertex merging, remeshing, or
decimation.  Every exported OBJ is reloaded and checked before the derivative
URDF and report are published.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import numpy as np

try:
    import trimesh
except ImportError as error:  # pragma: no cover - depends on host environment
    raise SystemExit(
        "trimesh is required; install it in the active project environment "
        "(for example: python3 -m pip install trimesh)"
    ) from error


PACKAGE_PREFIX = "package://rars01_description/"
PROFILE_OUTPUTS = {
    "train_obj": "go2_arm_dynamic_train_obj.urdf",
    "base_obj": "go2_arm_dynamic_base_obj.urdf",
}
BOUNDS_TOLERANCE = 1.0e-7
AREA_RELATIVE_TOLERANCE = 1.0e-6


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_single_mesh(path: Path) -> "trimesh.Trimesh":
    loaded = trimesh.load_mesh(path, process=False)
    if isinstance(loaded, trimesh.Scene):
        geometries = list(loaded.geometry.values())
        if len(geometries) != 1:
            raise ValueError(f"expected one mesh in {path}, found {len(geometries)}")
        loaded = geometries[0]
    if not isinstance(loaded, trimesh.Trimesh):
        raise TypeError(f"{path} did not load as a triangular mesh")
    if loaded.faces.ndim != 2 or loaded.faces.shape[1] != 3:
        raise ValueError(f"{path} contains non-triangular faces")
    return loaded


def _finite_mesh(mesh: "trimesh.Trimesh") -> bool:
    return bool(
        np.isfinite(mesh.vertices).all()
        and np.isfinite(mesh.faces).all()
        and np.isfinite(mesh.bounds).all()
        and np.isfinite(mesh.area)
    )


def _relative_area_error(before: float, after: float) -> float:
    if before == 0.0:
        return 0.0 if after == 0.0 else math.inf
    return abs(after - before) / abs(before)


def convert_and_validate(source: Path, output: Path) -> dict[str, Any]:
    source_mesh = _load_single_mesh(source)
    if not _finite_mesh(source_mesh):
        raise ValueError(f"source mesh has non-finite geometry: {source}")

    # Binary STL stores three coordinate records per face and has no shared
    # vertex index.  OBJ does.  Compact *exactly equal* coordinate rows into an
    # index (no rounding/tolerance and no Trimesh merge/process operation).
    # This is representation-only and substantially reduces MuJoCo's import
    # memory while preserving every ordered triangle byte-for-coordinate.
    unique_vertices, inverse = np.unique(
        np.asarray(source_mesh.vertices), axis=0, return_inverse=True
    )
    indexed_faces = inverse[np.asarray(source_mesh.faces)]
    export_mesh = trimesh.Trimesh(
        vertices=unique_vertices, faces=indexed_faces, process=False
    )

    # export_obj serializes the existing triangles; no Trimesh processing or
    # topology-changing helper is invoked anywhere in this pipeline.
    obj_text = trimesh.exchange.obj.export_obj(
        export_mesh, include_normals=False, include_color=False, digits=10
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=output.parent, suffix=".obj.tmp", delete=False
    ) as stream:
        temporary = Path(stream.name)
        stream.write(obj_text)
    os.replace(temporary, output)
    output.chmod(0o644)

    output_mesh = _load_single_mesh(output)
    source_faces = int(len(source_mesh.faces))
    output_faces = int(len(output_mesh.faces))
    source_bounds = np.asarray(source_mesh.bounds, dtype=np.float64)
    output_bounds = np.asarray(output_mesh.bounds, dtype=np.float64)
    source_extents = source_bounds[1] - source_bounds[0]
    output_extents = output_bounds[1] - output_bounds[0]
    source_area = float(source_mesh.area)
    output_area = float(output_mesh.area)
    area_error = _relative_area_error(source_area, output_area)
    triangle_coordinates_equal = bool(
        source_mesh.triangles.shape == output_mesh.triangles.shape
        and np.allclose(
            source_mesh.triangles,
            output_mesh.triangles,
            rtol=0.0,
            atol=BOUNDS_TOLERANCE,
        )
    )

    checks = {
        "finite_geometry": _finite_mesh(output_mesh),
        "face_count_equal": source_faces == output_faces,
        "ordered_triangle_coordinates_equal": triangle_coordinates_equal,
        "bounds_equal": bool(
            np.allclose(source_bounds, output_bounds, rtol=0.0, atol=BOUNDS_TOLERANCE)
        ),
        "extents_equal": bool(
            np.allclose(source_extents, output_extents, rtol=0.0, atol=BOUNDS_TOLERANCE)
        ),
        "area_equal": bool(area_error <= AREA_RELATIVE_TOLERANCE),
    }
    status = "PASS" if all(checks.values()) else "FAIL"
    record: dict[str, Any] = {
        "source": str(source),
        "output": str(output),
        "source_size_bytes": source.stat().st_size,
        "output_size_bytes": output.stat().st_size,
        "source_faces": source_faces,
        "output_faces": output_faces,
        "source_vertices": int(len(source_mesh.vertices)),
        "output_vertices": int(len(output_mesh.vertices)),
        "vertex_indexing": "exact-coordinate compaction (no rounding)",
        "source_bounds": source_bounds.tolist(),
        "output_bounds": output_bounds.tolist(),
        "source_extents": source_extents.tolist(),
        "output_extents": output_extents.tolist(),
        "source_area": source_area,
        "output_area": output_area,
        "relative_area_error": area_error,
        "source_sha256": sha256(source),
        "output_sha256": sha256(output),
        "checks": checks,
        "status": status,
    }
    if status != "PASS":
        raise ValueError(f"lossless conversion validation failed: {json.dumps(record, indent=2)}")
    return record


def resolve_package_mesh(filename: str, package_root: Path) -> Path | None:
    if not filename.startswith(PACKAGE_PREFIX):
        return None
    relative = filename[len(PACKAGE_PREFIX) :]
    resolved = (package_root / relative).resolve()
    root = package_root.resolve()
    if resolved != root and root not in resolved.parents:
        raise ValueError(f"mesh path escapes package root: {filename}")
    return resolved


def convert_urdf(
    urdf: Path, package_root: Path, output_root: Path, profile: str
) -> tuple[Path, list[dict[str, Any]]]:
    if profile not in PROFILE_OUTPUTS:
        raise ValueError(f"unknown profile {profile!r}")
    source_urdf = urdf.resolve()
    package_root = package_root.resolve()
    output_root = output_root.resolve()
    tree = ET.parse(source_urdf)
    root = tree.getroot()

    references: dict[Path, list[ET.Element]] = {}
    for mesh_element in root.findall(".//mesh"):
        filename = mesh_element.get("filename", "")
        source = resolve_package_mesh(filename, package_root)
        if source is None or source.suffix.lower() != ".stl":
            continue
        if not source.is_file():
            raise FileNotFoundError(f"referenced mesh does not exist: {source}")
        references.setdefault(source, []).append(mesh_element)
    if not references:
        raise ValueError(f"no RARS01 STL references found in {source_urdf}")

    assets = output_root / "assets" / "rars01"
    generated = output_root / "generated"
    assets.mkdir(parents=True, exist_ok=True)
    generated.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    used_output_names: dict[str, Path] = {}

    for source in sorted(references, key=lambda item: item.name.lower()):
        output_name = source.stem + ".obj"
        previous = used_output_names.setdefault(output_name.lower(), source)
        if previous != source:
            raise ValueError(f"OBJ filename collision: {previous} and {source}")
        output = assets / output_name
        record = convert_and_validate(source, output)
        record["source"] = source.relative_to(package_root).as_posix()
        record["output"] = output.relative_to(output_root).as_posix()
        record["reference_count"] = len(references[source])
        records.append(record)
        derivative_reference = Path("..") / "assets" / "rars01" / output_name
        for mesh_element in references[source]:
            mesh_element.set("filename", derivative_reference.as_posix())

    derivative = generated / PROFILE_OUTPUTS[profile]
    ET.indent(tree, space="  ")
    tree.write(derivative, encoding="utf-8", xml_declaration=True)

    # Report paths are workspace-relative by design; absolute developer paths
    # must never leak into generated runtime artifacts.
    report_path = output_root / "MESH_CONVERSION_REPORT.json"
    report = {
        "schema_version": 1,
        "converter": "trimesh.export_obj",
        "trimesh_version": trimesh.__version__,
        "process": False,
        "decimation": False,
        "bounds_tolerance": BOUNDS_TOLERANCE,
        "relative_area_tolerance": AREA_RELATIVE_TOLERANCE,
        "profile": profile,
        "source_urdf": source_urdf.relative_to(package_root).as_posix(),
        "derivative_urdf": derivative.relative_to(output_root).as_posix(),
        "unique_meshes": len(records),
        "records": records,
        "status": "PASS",
    }
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return derivative, records


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--profile", choices=sorted(PROFILE_OUTPUTS), required=True)
    args = parser.parse_args()

    derivative, records = convert_urdf(
        args.urdf, args.package_root, args.output_root, args.profile
    )
    mount = next((r for r in records if Path(r["source"]).stem == "arm_mount_link"), None)
    if mount is None:
        raise RuntimeError("arm_mount_link STL was not part of the conversion")
    print(
        f"{args.profile}: PASS, {len(records)} unique meshes -> {derivative}\n"
        f"arm_mount_link: {mount['source_faces']} -> {mount['output_faces']} faces, "
        f"area error={mount['relative_area_error']:.3e}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
