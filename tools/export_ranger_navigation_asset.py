#!/usr/bin/env python3
"""HH_261001 - Export a bounded realtime derivative of the approved Ranger FBX.

blender --background --factory-startup --python tools/export_ranger_navigation_asset.py

Original FBX/rig stays untouched. The separate cargo is display-only and makes
no load-capacity or simulator-validation claim.
"""

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys

import bpy
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
# HH_261001 - Offline authoring path only; runtime loads the committed GLB,
# not this external FBX. Adjust this path before regenerating on another host.
MODEL = Path("/home/hong/Downloads/ranger-carla-4ws-pipeline")
SOURCE = MODEL / "assets/fbx/ranger_carla_rigged_template.fbx"
MASK = MODEL / "config/ranger_rigid_weight_mask.v1.json"
OUTPUT = ROOT / "camrod_ui/camrod_ui_robot/assets/frontend/public/models/ranger-navigation.glb"
SOURCE_HASH = "ef93de43399af7b0a56bfb994fb66993d8ff9b4fc627de731dae0c08775fa77c"
WHEELS = {
    "Wheel_Front_Left": "wheel_fl", "Wheel_Front_Right": "wheel_fr",
    "Wheel_Rear_Left": "wheel_rl", "Wheel_Rear_Right": "wheel_rr",
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def material(name, color, metallic=0, roughness=0.6):
    result = bpy.data.materials.new(name)
    result.use_nodes = True
    result.diffuse_color = (*color, 1)
    shader = result.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Roughness"].default_value = roughness
    return result


def empty(name, parent=None, location=(0, 0, 0)):
    obj = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(obj)
    obj.parent = parent
    obj.location = location
    return obj


def gltf_vector(point):
    return [round(float(point[0]), 6), round(float(point[2]), 6), round(-float(point[1]), 6)]


def repair_weights():
    """Use the existing source-pinned authored repair, solely in memory."""
    repair_path = MODEL / "tools/blender_apply_ranger_rigid_weight_mask.py"
    sys.path.insert(0, str(repair_path.parent))
    spec = importlib.util.spec_from_file_location("ranger_weight_repair", repair_path)
    repair = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(repair)
    mask = repair.load_mask(MASK)
    obj, armature = repair.choose_objects()
    source = repair.validate_source(obj, armature, mask)
    loose = repair.build_loose_components(obj.data)
    operations, _ = repair.resolve_loose_operations(obj, loose, source["assignments"], mask)
    faces, _ = repair.resolve_face_operations(obj, source["assignments"], mask)
    for target, vertices in operations.items():
        if vertices:
            repair.assign_rigid(obj, vertices, target)
    before = len(obj.data.vertices)
    selected = repair.split_face_components(obj, faces)
    if len(obj.data.vertices) - before != mask["expected_output"]["duplicated_boundary_vertices"]:
        raise RuntimeError("Rigid-mask boundary split mismatch")
    repair.assign_rigid(obj, selected, "Vehicle_Base")
    assignments, counts = repair.rigid_assignments(obj)
    if counts != mask["expected_output"]["rigid_group_counts"]:
        raise RuntimeError("Rigid-mask assignment counts mismatch")
    pivots = {bone.name: armature.matrix_world @ bone.head_local for bone in armature.data.bones if bone.name in WHEELS}
    return obj, armature, assignments, pivots


def simplify(obj, maximum_triangles):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    # CAD flat plates carry many unnecessary coplanar triangles. Dissolve them
    # first; collapse only the remaining mesh to the explicit per-part budget.
    planar = obj.modifiers.new("Realtime planar simplification", "DECIMATE")
    planar.decimate_type = "DISSOLVE"
    planar.angle_limit = math.radians(0.5)
    planar.delimit = {"MATERIAL"}
    bpy.ops.object.modifier_apply(modifier=planar.name)
    obj.data.calc_loop_triangles()
    count = len(obj.data.loop_triangles)
    if count > maximum_triangles:
        collapse = obj.modifiers.new("Realtime triangle budget", "DECIMATE")
        collapse.ratio = maximum_triangles / count * 0.96
        collapse.use_collapse_triangulate = True
        bpy.ops.object.modifier_apply(modifier=collapse.name)
    obj.data.calc_loop_triangles()
    return len(obj.data.loop_triangles)


def add_box(name, center, size, mat, parent):
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    obj.parent = parent
    return obj


def conforming_wraps(root):
    """Project artwork onto existing body surfaces, not bounding-box planes."""
    body = bpy.data.objects["body"]
    bpy.context.view_layer.update()
    tree = BVHTree.FromObject(body, bpy.context.evaluated_depsgraph_get())
    configs = {
        # axis, outward sign, horizontal start/end, height start/end, min outer coordinate
        "left": (1, 1, 0.49, -0.69, 0.64, 0.98, 0.48),
        "right": (1, -1, -0.69, 0.49, 0.64, 0.98, 0.48),
        "front": (0, 1, -0.31, 0.31, 0.67, 0.94, 0.74),
        "rear": (0, -1, 0.48, -0.48, 0.64, 0.98, 0.74),
    }
    report = {}
    for name, (axis, sign, start, end, low, high, minimum) in configs.items():
        columns, rows = (32, 16) if name == "front" else (40, 20)
        horizontal = 0 if axis == 1 else 1
        vertices, uvs, valid, depths = [], [], [], []
        direction = Vector((0, 0, 0))
        direction[axis] = -sign
        for row in range(rows + 1):
            for column in range(columns + 1):
                u, v = column / columns, row / rows
                origin = Vector((0, 0, low + v * (high - low)))
                origin[axis] = sign * 2
                origin[horizontal] = start + u * (end - start)
                hit, _, _, _ = tree.ray_cast(origin, direction, 4.0)
                accepted = hit is not None and sign * hit[axis] >= minimum
                point = hit.copy() if hit is not None else origin.copy()
                point[axis] += sign * 0.002
                vertices.append(tuple(point))
                uvs.append((u, v))
                valid.append(accepted)
                depths.append(sign * hit[axis] if hit is not None else 0)
        faces = []
        for row in range(rows):
            for column in range(columns):
                a = row * (columns + 1) + column
                face = (a, a + 1, a + columns + 2, a + columns + 1)
                # Leave genuine cutouts/recesses white instead of stretching
                # one texture cell over empty space or protruding hardware.
                if not all(valid[index] for index in face) or max(depths[index] for index in face) - min(depths[index] for index in face) > 0.02:
                    continue
                for triangle in ((face[0], face[1], face[2]), (face[0], face[2], face[3])):
                    points = [Vector(vertices[index]) for index in triangle]
                    samples = [sum(points, Vector()) / 3]
                    samples.extend((points[index] + points[(index + 1) % 3]) / 2 for index in range(3))
                    samples.extend(points[index] * 0.6 + points[(index + 1) % 3] * 0.2 + points[(index + 2) % 3] * 0.2 for index in range(3))
                    supported = True
                    for sample in samples:
                        ray_origin = sample.copy()
                        ray_origin[axis] = sign * 2
                        hit, _, _, _ = tree.ray_cast(ray_origin, direction, 4.0)
                        gap = sign * (sample[axis] - hit[axis]) if hit is not None else float("inf")
                        if not 0 <= gap <= 0.008:
                            supported = False
                            break
                    if supported:
                        faces.append(triangle)
        data = bpy.data.meshes.new("wrap_" + name + "_mesh")
        data.from_pydata(vertices, [], faces)
        data.uv_layers.new(name="UVMap")
        for loop in data.loops:
            data.uv_layers.active.data[loop.index].uv = uvs[loop.vertex_index]
        obj = bpy.data.objects.new("wrap_" + name, data)
        bpy.context.scene.collection.objects.link(obj)
        obj.parent = root
        mat = material("sidewrap_" + name, (0.88, 0.9, 0.91), 0, 0.8)
        data.materials.append(mat)
        obj["runtime_texture_target"] = True
        obj["placement_note"] = "Ray-projected onto original LOD facade, 2 mm outward offset; cutouts excluded"
        report[name] = {
            "node": obj.name, "material": mat.name, "size_m": [abs(end - start), high - low],
            "height_range_m": [low, high], "surface_offset_m": 0.002,
            "projection": "Existing body LOD BVH, outward-facing surface; cutouts/recess jumps excluded",
            "grid_segments": [columns, rows], "faces": len(faces),
            "trimmed_triangles": columns * rows * 2 - len(faces),
            "surface_validation": "Seven centroid/edge/interior rays per triangle; retain only 0–8 mm outward clearance at every probe",
            "outward_direction": {"front": "+X", "rear": "-X", "left": "-Z", "right": "+Z"}[name],
        }
    return report


def finalize_glb():
    """Validate actual exported transforms/bounds and normalize bool extras."""
    with OUTPUT.open("rb") as stream:
        header = struct.unpack("<III", stream.read(12))
        length, kind = struct.unpack("<II", stream.read(8))
        gltf = json.loads(stream.read(length))
        binary_chunks = stream.read()
    if header[0] != 0x46546C67 or header[1] != 2 or kind != 0x4E4F534A:
        raise RuntimeError("Invalid GLB container")
    for node in gltf["nodes"]:
        for key in ("provisional", "default_visible", "runtime_texture_target"):
            if key in node.get("extras", {}):
                node["extras"][key] = bool(node["extras"][key])
    named = {node.get("name"): node for node in gltf["nodes"]}
    for name in ("ranger_navigation", "body", "wrap_left", "wrap_right", "wrap_front", "wrap_rear", "accessory_cargo_preview"):
        if name not in named:
            raise RuntimeError("Required runtime node missing: " + name)
    if "accessory_shelf_preview" in named:
        raise RuntimeError("Removed shelf must not remain in exported GLB")
    source_pivots = json.loads(MASK.read_text())["source"]["wheel_centers_cm"]
    for bone, name in WHEELS.items():
        for parent, child in ((name + "_steer", name + "_spin"), (name + "_spin", name)):
            if parent not in named or child not in {gltf["nodes"][index].get("name") for index in named[parent].get("children", [])}:
                raise RuntimeError("Wheel animation hierarchy mismatch: " + name)
        expected = np.array(gltf_vector(np.array(source_pivots[bone]) * 0.01))
        if not np.allclose(named[name + "_steer"]["translation"], expected, atol=1e-6):
            raise RuntimeError("Wheel pivot differs from source bone: " + name)
    corners = []

    def visit(index, translation, accessory=False):
        node = gltf["nodes"][index]
        if "matrix" in node or node.get("rotation", [0, 0, 0, 1]) != [0, 0, 0, 1] or node.get("scale", [1, 1, 1]) != [1, 1, 1]:
            raise RuntimeError("Unexpected non-translation node transform")
        translation = translation + np.array(node.get("translation", [0, 0, 0]))
        accessory = accessory or node.get("name") == "accessory_cargo_preview"
        if "mesh" in node and not accessory and not node.get("name", "").startswith("wrap_"):
            for primitive in gltf["meshes"][node["mesh"]]["primitives"]:
                accessor = gltf["accessors"][primitive["attributes"]["POSITION"]]
                corners.extend((np.array(accessor["min"]) + translation, np.array(accessor["max"]) + translation))
        for child in node.get("children", []):
            visit(child, translation, accessory)

    for index in gltf["scenes"][gltf.get("scene", 0)]["nodes"]:
        visit(index, np.zeros(3))
    points = np.array(corners)
    low, high = points.min(axis=0), points.max(axis=0)
    payload = json.dumps(gltf, separators=(",", ":")).encode()
    payload += b" " * (-len(payload) % 4)
    total = 12 + 8 + len(payload) + len(binary_chunks)
    OUTPUT.write_bytes(struct.pack("<III", header[0], 2, total) + struct.pack("<II", len(payload), kind) + payload + binary_chunks)
    triangles = sum(gltf["accessors"][primitive["indices"]]["count"] // 3 for mesh in gltf["meshes"] for primitive in mesh["primitives"])
    if triangles > 150000 or total > 6 * 1024 * 1024:
        raise RuntimeError("Realtime GLB exceeds bounded asset budget")
    return {
        "output_sha256": sha256(OUTPUT), "bytes": total, "triangles": triangles,
        "lod_bounds_m": {"minimum": low.tolist(), "maximum": high.tolist()},
        "lod_dimensions_m": {"length": float(high[0] - low[0]), "height": float(high[1] - low[1]), "width": float(high[2] - low[2])},
    }


def main():
    if sha256(SOURCE) != SOURCE_HASH:
        raise RuntimeError("Approved FBX checksum mismatch")
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=str(SOURCE), use_anim=False, use_image_search=False)
    bpy.context.view_layer.update()
    obj, armature, assignments, pivots_cm = repair_weights()
    print("Authored rigid-weight repair verified; separating render parts", flush=True)
    mesh = obj.data
    coordinates = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
    mesh.vertices.foreach_get("co", coordinates)
    coordinates = coordinates.reshape((-1, 3))
    world = np.array(obj.matrix_world, dtype=np.float32)
    coordinates = (coordinates @ world[:3, :3].T + world[:3, 3]) * 0.01
    bounds = (coordinates.min(axis=0), coordinates.max(axis=0))
    print("SOURCE_BOUNDS_METERS", [value.tolist() for value in bounds], flush=True)
    polygons = {name: [] for name in ("Vehicle_Base", *WHEELS)}
    for polygon in mesh.polygons:
        vertices = tuple(polygon.vertices)
        group = assignments[vertices[0]]
        if any(assignments[index] != group for index in vertices):
            raise RuntimeError("Polygon crosses rigid groups after approved repair")
        polygons[group].append(vertices)

    white = material("body_white", (0.88, 0.9, 0.91), 0.08, 0.36)
    metal = material("chassis_metal", (0.28, 0.31, 0.34), 0.7, 0.3)
    rubber = material("tire_rubber", (0.025, 0.028, 0.031), 0.0, 0.86)
    root = empty("ranger_navigation")
    root["source_geometry"] = "Approved Ranger FBX, derived realtime LOD"
    root["coordinate_contract"] = "meters; +X forward, +Y up, +Z right after glTF export"
    report_parts = {}
    wheel_report = {}
    for group, faces in polygons.items():
        name = WHEELS.get(group, "body")
        indices = np.unique(np.fromiter((index for face in faces for index in face), dtype=np.int32))
        mapping = np.full(len(coordinates), -1, dtype=np.int32)
        mapping[indices] = np.arange(len(indices))
        pivot = np.array(pivots_cm[group], dtype=np.float32) * 0.01 if group in WHEELS else np.zeros(3)
        vertices = coordinates[indices] - pivot
        local_faces = [tuple(int(mapping[index]) for index in face) for face in faces]
        data = bpy.data.meshes.new(name + "_mesh")
        data.from_pydata(vertices.tolist(), [], local_faces)
        data.update()
        part = bpy.data.objects.new(name, data)
        bpy.context.scene.collection.objects.link(part)
        for mat in (white, metal, rubber):
            data.materials.append(mat)
        centers = np.empty(len(data.polygons) * 3, dtype=np.float32)
        data.polygons.foreach_get("center", centers)
        centers = centers.reshape((-1, 3))
        colors = np.zeros(len(data.polygons), dtype=np.int32)
        if group in WHEELS:
            colors[:] = 1
            radial = np.hypot(centers[:, 0], centers[:, 2])
            colors[radial >= 0.095] = 2
            steer = empty(name + "_steer", root, pivot)
            spin = empty(name + "_spin", steer)
            part.parent = spin
            wheel_report[name] = {
                "steer_node": steer.name, "spin_node": spin.name,
                "pivot_m": gltf_vector(pivot), "steer_axis": "+Y", "spin_axis": "+Z",
                "rolling_radius_m": 0.153,
            }
        else:
            colors[centers[:, 2] < 0.36] = 1
            part.parent = root
        data.polygons.foreach_set("material_index", colors)
        triangle_count = simplify(part, 62000 if name == "body" else 14000)
        # Preserve CAD panel breaks while sharing normals on tires/rounded
        # hardware; fully flat triangles unnecessarily duplicate GLB vertices.
        part.data.polygons.foreach_set("use_smooth", np.ones(len(part.data.polygons), dtype=np.bool_))
        part.data.use_auto_smooth = True
        part.data.auto_smooth_angle = math.radians(30)
        report_parts[name] = {"triangles": triangle_count, "vertices": len(part.data.vertices)}
        print("LOD", name, report_parts[name], flush=True)

    # Delete only imported transient Blender scene objects, never source files.
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.objects.remove(armature, do_unlink=True)

    wrap_metadata = conforming_wraps(root)

    cargo = empty("accessory_cargo_preview", root)
    cargo["provisional"] = True
    cargo["default_visible"] = True
    cargo["description"] = "Display-only packed cardboard boxes inside bed; not measured load or physics"
    cardboard = (
        material("cargo_cardboard_kraft", (0.55, 0.34, 0.17), 0, 0.95),
        material("cargo_cardboard_light", (0.68, 0.45, 0.24), 0, 0.95),
        material("cargo_cardboard_dark", (0.45, 0.27, 0.13), 0, 0.95),
    )
    tape = material("cargo_packing_tape", (0.69, 0.51, 0.29), 0, 0.62)
    # The original bed floor is Y=0.615–0.616 m in the exported model. Four
    # close rows of three boxes fill its usable footprint while keeping small
    # separate seams between packages and clear space beneath the upper rails.
    floor_z = 0.616
    box_centers_x = (-0.56, -0.31, -0.06, 0.19)
    box_centers_y = (-0.275, 0.0, 0.275)
    box_heights = (
        (0.30, 0.33, 0.29),
        (0.33, 0.27, 0.31),
        (0.29, 0.32, 0.30),
        (0.31, 0.29, 0.33),
    )
    box_count = 0
    for row, x in enumerate(box_centers_x):
        for column, y in enumerate(box_centers_y):
            box_count += 1
            size = (0.236 + 0.004 * ((row + column) % 2),
                    0.252 + 0.004 * ((row + 2 * column) % 2),
                    box_heights[row][column])
            add_box("cargo_box_" + str(box_count),
                    (x, y, floor_z + size[2] / 2), size,
                    cardboard[(row + 2 * column) % len(cardboard)], cargo)
            add_box("cargo_box_tape_" + str(box_count),
                    (x, y, floor_z + size[2] + 0.001),
                    (0.032, size[1], 0.002), tape, cargo)
    cargo_top_z = floor_z + max(max(row) for row in box_heights) + 0.002
    bpy.context.view_layer.update()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(OUTPUT), export_format="GLB", export_yup=True,
        export_apply=True, export_animations=False, export_skins=False,
        export_extras=True, export_cameras=False, export_lights=False,
        export_texcoords=True, export_normals=True, export_materials="EXPORT",
    )
    if sha256(SOURCE) != SOURCE_HASH:
        raise RuntimeError("Source changed during GLB export")
    export_info = finalize_glb()
    report = {
        "asset": OUTPUT.name, "source": str(SOURCE), "source_sha256": SOURCE_HASH,
        "source_unchanged": True, **export_info, "parts": report_parts,
        "coordinate_contract": "meters; +X forward, +Y up, +Z right; ROS (x,y,z) -> glTF (x,z,-y)",
        "source_bounds_m": {
            "minimum": [float(bounds[0][0]), float(bounds[0][2]), float(-bounds[1][1])],
            "maximum": [float(bounds[1][0]), float(bounds[1][2]), float(-bounds[0][1])],
        },
        "body_dimensions_m": {"length": float(bounds[1][0] - bounds[0][0]), "width": float(bounds[1][1] - bounds[0][1]), "height": float(bounds[1][2] - bounds[0][2])},
        "wheels": wheel_report, "wraps": wrap_metadata,
        "cargo": {"node": cargo.name, "provisional": True, "default_visible": True, "contents": ["12 packed cardboard boxes"], "box_count": box_count, "bed_floor_height_m": floor_z, "maximum_height_m": cargo_top_z, "footprint_m": {"x": [-0.68, 0.31], "lateral": [-0.403, 0.403]}, "note": "Display-only cargo resting on the original bed floor and below upper rails; no physics, load-capacity or real cargo measurement claim. Upper shelf removed entirely."},
        "derivation": "Existing source-pinned rigid weight repair in memory, split rigid wheel/base groups, planar dissolve and bounded collapse decimation. White/dark-rubber/metal display materials replace absent source materials.",
        "rigid_weight_mask_sha256": sha256(MASK),
        "blender_version": bpy.app.version_string,
        "reproduce": "blender --background --factory-startup --python tools/export_ranger_navigation_asset.py",
        "not_evidence": "Display asset only; no road, simulator, load capacity or kinematic acceptance claim.",
    }
    OUTPUT.with_suffix(".provenance.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    if "--finalize-existing" in sys.argv:
        # Mechanical JSON/bounds validation only; no source import or geometry change.
        report_path = OUTPUT.with_suffix(".provenance.json")
        report = json.loads(report_path.read_text())
        report.update(finalize_glb())
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
    else:
        main()
