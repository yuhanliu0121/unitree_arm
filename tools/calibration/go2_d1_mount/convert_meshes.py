"""Convert robot meshes to axis-stable PLY files for Blender assembly."""

from __future__ import annotations

from pathlib import Path

import trimesh


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
GO2_SOURCE = HERE / "vendor" / "go2_description" / "dae"
D1_SOURCE = WORKSPACE / "src" / "d1_description" / "meshes"
OUTPUT = HERE / "converted"


def convert_directory(source: Path, pattern: str, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    for input_path in sorted(source.glob(pattern)):
        scene = trimesh.load_scene(input_path, process=False)
        if not scene.geometry:
            raise RuntimeError(f"No geometry loaded from {input_path}")
        # Bake the source scene graph into vertices. PLY has no up-axis
        # metadata, so Blender receives the exact ROS-space coordinates rather
        # than applying format-dependent glTF/Collada axis conversions.
        mesh = scene.to_geometry()
        output_path = output / f"{input_path.stem}.ply"
        mesh.export(output_path, file_type="ply")
        print(
            f"{input_path.name} -> {output_path.relative_to(HERE)} "
            f"geometry={len(scene.geometry)} bounds={mesh.bounds.tolist()}"
        )


def main() -> None:
    convert_directory(GO2_SOURCE, "*.dae", OUTPUT / "go2")
    convert_directory(D1_SOURCE, "*.STL", OUTPUT / "d1")


if __name__ == "__main__":
    main()
