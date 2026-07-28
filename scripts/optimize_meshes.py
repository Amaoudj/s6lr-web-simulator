"""Build browser-friendly copies of the two repeated high-density servo meshes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh


DEFAULT_TARGETS = {
    "Servo femur.stl": 24_000,
    "Servo tibia.stl": 24_000,
}


def mesh_stats(mesh: trimesh.Trimesh, path: Path) -> dict[str, object]:
    return {
        "file": path.name,
        "faces": int(len(mesh.faces)),
        "vertices": int(len(mesh.vertices)),
        "bytes": path.stat().st_size,
        "bounds_mm": np.asarray(mesh.bounds).round(5).tolist(),
    }


def simplify(source: Path, destination: Path, face_count: int) -> dict[str, object]:
    original = trimesh.load_mesh(source, file_type="stl", process=True)
    if not isinstance(original, trimesh.Trimesh):
        raise TypeError(f"{source} did not load as a single mesh")

    optimized = original.simplify_quadric_decimation(face_count=face_count)
    destination.parent.mkdir(parents=True, exist_ok=True)
    optimized.export(destination, file_type="stl")

    reloaded = trimesh.load_mesh(destination, file_type="stl", process=False)
    if not isinstance(reloaded, trimesh.Trimesh):
        raise TypeError(f"{destination} did not reload as a single mesh")

    original_extent = np.asarray(original.extents)
    optimized_extent = np.asarray(reloaded.extents)
    extent_delta = np.abs(original_extent - optimized_extent)
    if np.any(extent_delta > 0.05):
        raise ValueError(
            f"{source.name} changed bounds by more than 0.05 mm: "
            f"{extent_delta.tolist()}"
        )

    return {
        "source": mesh_stats(original, source),
        "optimized": mesh_stats(reloaded, destination),
        "max_extent_delta_mm": float(extent_delta.max()),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=Path("robot-source/meshes"),
        help="Directory containing the untouched source STL files.",
    )
    parser.add_argument(
        "--destination",
        type=Path,
        default=Path("public/models/s6lr/xml_Files"),
        help="Directory containing the browser-served STL files.",
    )
    args = parser.parse_args()

    results = []
    for name, face_count in DEFAULT_TARGETS.items():
        source = args.source / name
        destination = args.destination / name
        if not source.is_file():
            raise FileNotFoundError(source)
        results.append(simplify(source, destination, face_count))

    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
