#!/usr/bin/env python3

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = [
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr],
    ]
    ry = [
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp],
    ]
    rz = [
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1],
    ]
    return matmul(matmul(rz, ry), rx)


def matmul(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)] for r in range(3)]


def transpose(m):
    return [[m[c][r] for c in range(3)] for r in range(3)]


def ros_to_unity_position(v):
    return [-v[1], v[2], v[0]]


def matrix_to_quaternion(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def ros_rpy_to_unity_quaternion(rpy):
    ros_rot = rpy_to_matrix(*rpy)
    ros_to_unity = [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
    ]
    unity_to_ros = transpose(ros_to_unity)
    unity_rot = matmul(matmul(ros_to_unity, ros_rot), unity_to_ros)
    return matrix_to_quaternion(unity_rot)


def parse_xyz_rpy(origin_elem):
    if origin_elem is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = [float(v) for v in origin_elem.attrib.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin_elem.attrib.get("rpy", "0 0 0").split()]
    return xyz, rpy


def build_world_transforms(root_link, child_by_parent):
    world_transforms = {root_link: ([0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])}
    queue = [root_link]
    while queue:
        parent = queue.pop(0)
        parent_pos, parent_rot = world_transforms[parent]
        for joint in child_by_parent.get(parent, []):
            child = joint["child"]
            j_pos = joint["xyz"]
            j_rot = rpy_to_matrix(*joint["rpy"])

            child_rot = matmul(parent_rot, j_rot)
            child_pos = [
                parent_pos[0] + parent_rot[0][0] * j_pos[0] + parent_rot[0][1] * j_pos[1] + parent_rot[0][2] * j_pos[2],
                parent_pos[1] + parent_rot[1][0] * j_pos[0] + parent_rot[1][1] * j_pos[1] + parent_rot[1][2] * j_pos[2],
                parent_pos[2] + parent_rot[2][0] * j_pos[0] + parent_rot[2][1] * j_pos[1] + parent_rot[2][2] * j_pos[2],
            ]
            world_transforms[child] = (child_pos, child_rot)
            queue.append(child)
    return world_transforms


def main():
    repo_root = Path(__file__).resolve().parents[2]
    urdf_path = repo_root / "src/mirobot_description/urdf/mirobot_urdf_2.urdf"
    output_dir = repo_root / "media/unity/mirobot_urdf_export"
    output_dir.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(urdf_path)
    robot = tree.getroot()

    links = {}
    for link in robot.findall("link"):
        link_name = link.attrib["name"]
        visual = link.find("visual")
        mesh_path = None
        color = [0.8, 0.8, 0.8, 1.0]
        if visual is not None:
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                mesh_path = mesh.attrib.get("filename", "")
            material = visual.find("material")
            if material is not None:
                color_elem = material.find("color")
                if color_elem is not None:
                    color = [float(v) for v in color_elem.attrib.get("rgba", "0.8 0.8 0.8 1").split()]
        links[link_name] = {
            "mesh": mesh_path,
            "rgba": color,
        }

    joints = []
    child_by_parent = {}
    parent_by_child = {}
    for joint in robot.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        origin = joint.find("origin")
        xyz, rpy = parse_xyz_rpy(origin)
        joint_data = {
            "name": joint.attrib.get("name", ""),
            "type": joint.attrib.get("type", "fixed"),
            "parent": parent.attrib["link"],
            "child": child.attrib["link"],
            "xyz": xyz,
            "rpy": rpy,
        }
        joints.append(joint_data)
        child_by_parent.setdefault(joint_data["parent"], []).append(joint_data)
        parent_by_child[joint_data["child"]] = joint_data

    root_link = "world" if "world" in links else "base_link"
    world_tf = build_world_transforms(root_link, child_by_parent)

    hierarchy = {
        "root_link": root_link,
        "units": "meters",
        "coordinate_system": "Unity (X right, Y up, Z forward)",
        "source_coordinate_system": "ROS URDF (X forward, Y left, Z up)",
        "links": [],
        "joints": joints,
        "note": "No texture files are referenced in this URDF. Colors are imported from visual/material/color rgba.",
    }

    for link_name, data in links.items():
        parent_joint = parent_by_child.get(link_name)
        parent_link = parent_joint["parent"] if parent_joint else None
        local_xyz = parent_joint["xyz"] if parent_joint else [0.0, 0.0, 0.0]
        local_rpy = parent_joint["rpy"] if parent_joint else [0.0, 0.0, 0.0]
        world_pos, _ = world_tf.get(link_name, ([0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        local_xyz_unity = ros_to_unity_position(local_xyz)
        world_xyz_unity = ros_to_unity_position(world_pos)
        local_quat_unity = ros_rpy_to_unity_quaternion(local_rpy)

        fbx_name = None
        if data["mesh"] and data["mesh"].endswith(".STL"):
            fbx_name = Path(data["mesh"]).name.replace(".STL", ".fbx")

        hierarchy["links"].append(
            {
                "name": link_name,
                "parent": parent_link,
                "local_xyz_ros": local_xyz,
                "local_rpy_ros": local_rpy,
                "local_xyz_unity": local_xyz_unity,
                "local_quat_unity_xyzw": local_quat_unity,
                "world_xyz_ros": world_pos,
                "world_xyz_unity": world_xyz_unity,
                "mesh_package_path": data["mesh"],
                "fbx_file": fbx_name,
                "material_rgba": data["rgba"],
            }
        )

    with (output_dir / "mirobot_hierarchy.json").open("w", encoding="utf-8") as f:
        json.dump(hierarchy, f, indent=2)

    unity_cs = r'''using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[Serializable]
public class MirobotLink
{
    public string name;
    public string parent;
    public float[] local_xyz_unity;
    public float[] local_quat_unity_xyzw;
    public string fbx_file;
    public float[] material_rgba;
}

[Serializable]
public class MirobotHierarchy
{
    public string root_link;
    public MirobotLink[] links;
}

public static class MirobotImporter
{
    [MenuItem("Mirobot/Build From URDF JSON")]
    public static void Build()
    {
        string baseFolder = "Assets/Mirobot";
        string jsonPath = Path.Combine(baseFolder, "mirobot_hierarchy.json");
        if (!File.Exists(jsonPath))
        {
            Debug.LogError($"Missing {jsonPath}. Copy it from media/unity/mirobot_urdf_export.");
            return;
        }

        var hierarchy = JsonUtility.FromJson<MirobotHierarchy>(File.ReadAllText(jsonPath));
        var root = new GameObject("Mirobot_URDF");
        var created = new Dictionary<string, GameObject>();

        foreach (var link in hierarchy.links)
        {
            var go = new GameObject(link.name);
            created[link.name] = go;
        }

        foreach (var link in hierarchy.links)
        {
            var go = created[link.name];
            if (string.IsNullOrEmpty(link.parent) || !created.ContainsKey(link.parent))
            {
                go.transform.SetParent(root.transform, false);
            }
            else
            {
                go.transform.SetParent(created[link.parent].transform, false);
            }

            go.transform.localPosition = new Vector3(link.local_xyz_unity[0], link.local_xyz_unity[1], link.local_xyz_unity[2]);
            go.transform.localRotation = new Quaternion(
                link.local_quat_unity_xyzw[0],
                link.local_quat_unity_xyzw[1],
                link.local_quat_unity_xyzw[2],
                link.local_quat_unity_xyzw[3]
            );

            if (!string.IsNullOrEmpty(link.fbx_file))
            {
                string meshPath = Path.Combine(baseFolder, "fbx", link.fbx_file).Replace("\\", "/");
                var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(meshPath);
                if (prefab != null)
                {
                    var visual = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                    visual.transform.SetParent(go.transform, false);

                    var renderers = visual.GetComponentsInChildren<MeshRenderer>(true);
                    if (renderers != null && renderers.Length > 0)
                    {
                        var mat = new Material(Shader.Find("Standard"));
                        if (link.material_rgba != null && link.material_rgba.Length == 4)
                        {
                            mat.color = new Color(link.material_rgba[0], link.material_rgba[1], link.material_rgba[2], link.material_rgba[3]);
                        }
                        foreach (var renderer in renderers)
                        {
                            renderer.sharedMaterial = mat;
                        }
                    }
                }
                else
                {
                    Debug.LogWarning($"Mesh not found at {meshPath}");
                }
            }
        }

        Debug.Log("Mirobot hierarchy built.");
    }
}
'''

    with (output_dir / "MirobotImporter.cs").open("w", encoding="utf-8") as f:
        f.write(unity_cs)

    readme = """Mirobot Unity URDF Export

Generated files:
- mirobot_hierarchy.json: URDF link/joint hierarchy, local transforms, FBX part mapping, and RGBA materials.
- MirobotImporter.cs: Unity Editor menu tool to build GameObject hierarchy from the JSON.

How to use in Unity:
1. Create folder Assets/Mirobot/fbx and copy all FBX parts there.
2. Copy mirobot_hierarchy.json to Assets/Mirobot/.
3. Copy MirobotImporter.cs into Assets/Editor/.
4. In Unity menu, run Mirobot -> Build From URDF JSON.

Notes:
- URDF in this repository defines colors but does not reference external texture image files.
- If you need textured materials, add texture maps manually in Unity after import.
"""

    with (output_dir / "README.txt").open("w", encoding="utf-8") as f:
        f.write(readme)

    print(f"Generated Unity export assets in: {output_dir}")


if __name__ == "__main__":
    main()
