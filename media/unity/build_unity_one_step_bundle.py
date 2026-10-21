#!/usr/bin/env python3

import argparse
import shutil
import subprocess
from pathlib import Path


def run(cmd, cwd=None):
    subprocess.run(cmd, cwd=cwd, check=True)


def ensure_fbx(repo_root: Path):
    meshes_dir = repo_root / "src/mirobot_description/meshes"
    fbx_dir = repo_root / "media/unity/mirobot_fbx"
    fbx_dir.mkdir(parents=True, exist_ok=True)

    assimp = shutil.which("assimp")
    if assimp is None:
        raise RuntimeError("assimp not found. Install with: sudo apt install assimp-utils")

    stl_files = sorted(meshes_dir.glob("*.STL"))
    if not stl_files:
        raise RuntimeError(f"No STL files found in {meshes_dir}")

    for stl in stl_files:
        out_fbx = fbx_dir / (stl.stem + ".fbx")
        run([assimp, "export", str(stl), str(out_fbx)])

    return fbx_dir


def generate_urdf_assets(repo_root: Path):
    generator = repo_root / "media/unity/generate_mirobot_unity_assets.py"
    run(["python3", str(generator)])
    out_dir = repo_root / "media/unity/mirobot_urdf_export"
    required = [
        out_dir / "mirobot_hierarchy.json",
        out_dir / "MirobotImporter.cs",
    ]
    for path in required:
        if not path.exists():
            raise RuntimeError(f"Missing generated file: {path}")
    return out_dir


def build_bundle(repo_root: Path, fbx_dir: Path, urdf_export_dir: Path):
    bundle_assets = repo_root / "media/unity/one_step_bundle/Assets"
    mirobot_dir = bundle_assets / "Mirobot"
    editor_dir = bundle_assets / "Editor"
    mirobot_fbx_dir = mirobot_dir / "fbx"

    if bundle_assets.parent.exists():
        shutil.rmtree(bundle_assets.parent)

    mirobot_fbx_dir.mkdir(parents=True, exist_ok=True)
    editor_dir.mkdir(parents=True, exist_ok=True)

    for fbx in sorted(fbx_dir.glob("*.fbx")):
        shutil.copy2(fbx, mirobot_fbx_dir / fbx.name)

    shutil.copy2(urdf_export_dir / "mirobot_hierarchy.json", mirobot_dir / "mirobot_hierarchy.json")
    shutil.copy2(urdf_export_dir / "MirobotImporter.cs", editor_dir / "MirobotImporter.cs")

    readme = bundle_assets.parent / "README.txt"
    readme.write_text(
        """One-step Unity bundle

This folder is ready to copy into a Unity project root.

Contents:
- Assets/Mirobot/fbx/*.fbx
- Assets/Mirobot/mirobot_hierarchy.json
- Assets/Editor/MirobotImporter.cs

Usage:
1. Copy the Assets folder into your Unity project root.
2. In Unity, wait for import/compile.
3. Run menu: Mirobot -> Build From URDF JSON

Coordinate system:
- Transforms are pre-converted from ROS URDF (X forward, Y left, Z up)
  to Unity (X right, Y up, Z forward).

Materials/textures:
- URDF RGBA colors are applied to generated materials.
- No texture images are defined in this URDF, so textured maps are not auto-assigned.
""",
        encoding="utf-8",
    )

    return bundle_assets.parent


def copy_to_unity_project(bundle_dir: Path, unity_project: Path):
    src_assets = bundle_dir / "Assets"
    dst_assets = unity_project / "Assets"
    if not unity_project.exists() or not dst_assets.exists():
        raise RuntimeError(f"Unity project path invalid (missing Assets folder): {unity_project}")

    shutil.copytree(src_assets / "Mirobot", dst_assets / "Mirobot", dirs_exist_ok=True)
    (dst_assets / "Editor").mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_assets / "Editor/MirobotImporter.cs", dst_assets / "Editor/MirobotImporter.cs")


def main():
    parser = argparse.ArgumentParser(description="Build one-step Unity import bundle for Mirobot")
    parser.add_argument(
        "--unity-project",
        type=Path,
        help="Optional path to Unity project root (must contain Assets). If provided, files are copied directly.",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]

    fbx_dir = ensure_fbx(repo_root)
    urdf_export_dir = generate_urdf_assets(repo_root)
    bundle_dir = build_bundle(repo_root, fbx_dir, urdf_export_dir)

    if args.unity_project:
        copy_to_unity_project(bundle_dir, args.unity_project.resolve())
        print(f"Bundle copied directly into Unity project: {args.unity_project.resolve()}")
    else:
        print(f"Bundle created at: {bundle_dir}")


if __name__ == "__main__":
    main()
