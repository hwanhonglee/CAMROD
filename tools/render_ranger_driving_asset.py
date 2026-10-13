#!/usr/bin/env python3
"""HH_261001 - Render the approved Ranger FBX as a transparent passive UI asset.

Run with Blender, not CPython:
  blender --background --factory-startup --python tools/render_ranger_driving_asset.py

Only the output PNG/provenance are written. No source FBX, rig, pose or mesh
topology is edited/exported. This is a studio illustration, not driving evidence.
"""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

import bpy
from mathutils import Vector
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
# HH_261001 - This external source is needed only for offline regeneration;
# deployed UI assets are committed and do not read this host-specific path.
SOURCE = Path("/home/hong/Downloads/ranger-carla-4ws-pipeline/assets/fbx/ranger_carla_rigged_template.fbx")
EXPECTED_HASH = "ef93de43399af7b0a56bfb994fb66993d8ff9b4fc627de731dae0c08775fa77c"
OUTPUT = ROOT / "camrod_ui/camrod_ui_robot/assets/frontend/public/models/ranger-driving-rear.png"

# This host's minimal Blender package lacks its bundled OCIO configuration.
# Bootstrap a temporary explicit linear-to-display transform before Blender
# starts; do not repair or modify any host installation files.
OCIO_CONFIG = """ocio_profile_version: 1
search_path: ""
strictparsing: true
roles:
  default: Linear
  scene_linear: Linear
  data: Raw
  color_picking: sRGB
  texture_paint: sRGB
  default_byte: sRGB
  default_float: Linear
  color_timing: Linear
  compositing_log: Linear
displays:
  sRGB:
    - !<View> {name: Standard, colorspace: sRGB}
active_displays: [sRGB]
active_views: [Standard]
colorspaces:
  - !<ColorSpace>
    name: Linear
    family: linear
    bitdepth: 32f
    isdata: false
    allocation: uniform
  - !<ColorSpace>
    name: sRGB
    family: display
    bitdepth: 8ui
    isdata: false
    allocation: uniform
    from_reference: !<ExponentTransform> {value: [0.45454545, 0.45454545, 0.45454545, 1]}
  - !<ColorSpace>
    name: Raw
    family: data
    bitdepth: 32f
    isdata: true
    allocation: uniform
"""


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def look_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()


def material(name, color, metallic, roughness):
    result = bpy.data.materials.new(name)
    result.use_nodes = True
    shader = result.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Roughness"].default_value = roughness
    return result


def main():
    if not os.environ.get("RANGER_RENDER_OCIO_READY"):
        with tempfile.TemporaryDirectory(prefix="ranger-render-color-") as temporary:
            config = Path(temporary) / "config.ocio"
            config.write_text(OCIO_CONFIG)
            environment = dict(os.environ, OCIO=str(config), RANGER_RENDER_OCIO_READY="1")
            subprocess.run([
                bpy.app.binary_path, "--background", "--factory-startup", "--threads", "8",
                "--python", str(Path(__file__).resolve()),
            ], env=environment, check=True)
        return
    source_hash = sha256(SOURCE)
    if source_hash != EXPECTED_HASH:
        raise RuntimeError("Approved Ranger checkpoint hash mismatch")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.import_scene.fbx(filepath=str(SOURCE))
    meshes = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    source_counts = {obj.name: {"vertices": len(obj.data.vertices), "polygons": len(obj.data.polygons)} for obj in meshes}
    material_fallback = []
    for obj in meshes:
        if len(obj.data.materials):
            continue
        material_fallback.append(obj.name)
        for mat in (
            material("Display fallback olive powder coat", (0.19, 0.235, 0.075), 0.22, 0.42),
            material("Display fallback warm alloy chassis", (0.40, 0.37, 0.24), 0.65, 0.3),
            material("Display fallback tire rubber", (0.023, 0.027, 0.024), 0.02, 0.78),
        ):
            obj.data.materials.append(mat)
        # The source FBX contains no material slots. Presentation-only colors
        # use existing geometric regions, with wheel centers from its rig.
        centers = np.empty(len(obj.data.polygons) * 3, dtype=np.float32)
        obj.data.polygons.foreach_get("center", centers)
        centers = centers.reshape((-1, 3))
        transform = np.array(obj.matrix_world, dtype=np.float32)
        centers = centers @ transform[:3, :3].T + transform[:3, 3]
        indices = np.zeros(len(obj.data.polygons), dtype=np.int32)
        indices[centers[:, 2] < 36] = 1
        for armature in [item for item in bpy.context.scene.objects if item.type == "ARMATURE"]:
            for bone in armature.data.bones:
                if not bone.name.startswith("Wheel_"):
                    continue
                center = armature.matrix_world @ bone.head_local
                radial = np.hypot(centers[:, 0] - center.x, centers[:, 2] - center.z)
                rubber = (radial >= 9.5) & (radial <= 16) & (np.abs(centers[:, 1] - center.y) <= 8)
                indices[rubber] = 2
        obj.data.polygons.foreach_set("material_index", indices)

    # Source is centimeters, +X front / +Y left / +Z up. Scaling the scene's
    # imported roots preserves mesh coordinates, topology and the neutral rig.
    imported_roots = [obj for obj in bpy.context.scene.objects if obj.parent is None]
    unit_root = bpy.data.objects.new("Render-only centimeter conversion", None)
    bpy.context.scene.collection.objects.link(unit_root)
    for obj in imported_roots:
        obj.parent = unit_root
    unit_root.scale = (0.01, 0.01, 0.01)
    bpy.context.view_layer.update()
    corners = [obj.matrix_world @ Vector(corner) for obj in meshes for corner in obj.bound_box]
    low = Vector(tuple(min(point[axis] for point in corners) for axis in range(3)))
    high = Vector(tuple(max(point[axis] for point in corners) for axis in range(3)))
    center = (low + high) * 0.5
    span = max(high - low)
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 16
    # No compositor or optional denoiser is needed for this small studio asset.
    # Minimal host builds can have unavailable OIDN/color-management resources.
    scene.cycles.use_denoising = False
    scene.use_nodes = False
    scene.render.use_compositing = False
    scene.render.use_sequencer = False
    scene.render.resolution_x = 1000
    scene.render.resolution_y = 1000
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = True
    scene.render.image_settings.file_format = "OPEN_EXR"
    scene.render.image_settings.color_mode = "RGBA"
    scene.world.use_nodes = True
    scene.world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.8, 0.87, 1, 1)
    scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.5
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.exposure = 0
    scene.view_settings.gamma = 1

    camera_data = bpy.data.cameras.new("Rear elevated display camera")
    camera = bpy.data.objects.new("Rear elevated display camera", camera_data)
    scene.collection.objects.link(camera)
    # Rear-right elevated: forward projects toward the upper side of the UI.
    camera.location = center + Vector((-3.4, -1.5, 2.3)) * span
    look_at(camera, center)
    camera_data.type = "ORTHO"
    camera_data.ortho_scale = span * 1.48
    scene.camera = camera
    for name, relative, energy, size in (
        ("Soft key", (-2.3, -3, 4), 650, 4),
        ("Soft fill", (1, 2, 2), 420, 3),
        ("Roof rim", (2, -1, 4), 800, 2.5),
    ):
        light_data = bpy.data.lights.new(name, "AREA")
        light_data.energy = energy
        light_data.shape = "DISK"
        light_data.size = size
        light = bpy.data.objects.new(name, light_data)
        scene.collection.objects.link(light)
        light.location = center + Vector(relative)
        look_at(light, center)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    # Preserve native linear RGBA and use a single explicit display encoding.
    # The host produced invalid output with its optional denoiser/compositor;
    # both are disabled above and the resulting PNG is validated below.
    with tempfile.TemporaryDirectory(prefix="ranger-render-linear-") as temporary:
        linear = Path(temporary) / "ranger.exr"
        candidate = Path(temporary) / "ranger.png"
        scene.render.filepath = str(linear)
        bpy.ops.render.render(write_still=True)
        subprocess.run([
            "convert", str(linear), "-set", "colorspace", "RGB", "-colorspace", "sRGB",
            "-depth", "8", "PNG32:" + str(candidate),
        ], check=True)
        values = subprocess.check_output([
            "identify", "-format", "%[fx:maxima.r] %[fx:minima.a]", str(candidate),
        ], text=True).split()
        if float(values[0]) < 0.05 or float(values[1]) > 0.05:
            raise RuntimeError("Renderer produced an empty/opaque image; existing UI asset was not replaced")
        OUTPUT.write_bytes(candidate.read_bytes())
    assert sha256(SOURCE) == source_hash, "Source changed during render"
    provenance = {
        "asset": OUTPUT.name,
        "description": "Transparent studio render of actual approved Ranger FBX mesh; not live or road-test evidence.",
        "source": str(SOURCE), "source_sha256": source_hash,
        "source_selection": "docs/MODEL_BUILD_CURRENT.md: approved template checkpoint",
        "output_sha256": sha256(OUTPUT), "blender_version": bpy.app.version_string,
        "source_geometry": source_counts,
        "geometry_modified": False, "source_fbx_modified": False,
        "source_axes": "+X front, +Y left, +Z up; centimeters",
        "camera": "rear-right elevated orthographic, approximately 32 degrees down",
        "material_fallback_meshes": material_fallback,
        "material_note": "Source mesh has no material slots. Olive powder coat, warm alloy lower chassis and dark tire rubber are display-only fallback materials, assigned by existing geometric regions; not claimed source textures.",
        "render_command": "blender --background --factory-startup --python tools/render_ranger_driving_asset.py",
        "color_management": "Native linear RGBA OpenEXR render encoded once to sRGB PNG by ImageMagick.",
        "render_engine": "Cycles CPU, 16 samples; optional denoising and compositing disabled after output validation failed with them enabled.",
    }
    OUTPUT.with_suffix(".provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps(provenance, indent=2))


if __name__ == "__main__":
    main()
