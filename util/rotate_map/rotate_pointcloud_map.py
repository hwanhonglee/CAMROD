#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os
import struct
from typing import List, Tuple, Dict, Any

import numpy as np


# ============================================================
# Common
# ============================================================

def make_transform(roll_deg: float, pitch_deg: float, yaw_deg: float,
                   tx: float, ty: float, tz: float):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    # ZYX rotation: Rz(yaw) * Ry(pitch) * Rx(roll)
    Rz = np.array([[cy, -sy, 0.0],
                   [sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]], dtype=np.float64)
    Ry = np.array([[ cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]], dtype=np.float64)
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr,  cr]], dtype=np.float64)

    R = Rz @ Ry @ Rx
    t = np.array([tx, ty, tz], dtype=np.float64)
    return R, t


def apply_transform_to_xyz(arr: np.ndarray, R: np.ndarray, t: np.ndarray):
    required = ("x", "y", "z")
    if not all(name in arr.dtype.names for name in required):
        raise ValueError(f"Point cloud does not contain required XYZ fields: {required}")

    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    xyz_t = (xyz @ R.T) + t

    arr["x"] = xyz_t[:, 0].astype(arr.dtype["x"])
    arr["y"] = xyz_t[:, 1].astype(arr.dtype["y"])
    arr["z"] = xyz_t[:, 2].astype(arr.dtype["z"])

    # Rotate normals too if present
    normal_candidates = [
        ("normal_x", "normal_y", "normal_z"),
        ("nx", "ny", "nz"),
    ]
    for nx, ny, nz in normal_candidates:
        if all(name in arr.dtype.names for name in (nx, ny, nz)):
            normals = np.stack([arr[nx], arr[ny], arr[nz]], axis=1).astype(np.float64)
            normals_t = normals @ R.T
            arr[nx] = normals_t[:, 0].astype(arr.dtype[nx])
            arr[ny] = normals_t[:, 1].astype(arr.dtype[ny])
            arr[nz] = normals_t[:, 2].astype(arr.dtype[nz])


# ============================================================
# PCD
# ============================================================

PCD_TYPE_TO_DTYPE = {
    ("F", 4): np.float32,
    ("F", 8): np.float64,
    ("I", 1): np.int8,
    ("I", 2): np.int16,
    ("I", 4): np.int32,
    ("I", 8): np.int64,
    ("U", 1): np.uint8,
    ("U", 2): np.uint16,
    ("U", 4): np.uint32,
    ("U", 8): np.uint64,
}

DTYPE_TO_PCD = {
    np.dtype(np.float32): ("F", 4),
    np.dtype(np.float64): ("F", 8),
    np.dtype(np.int8): ("I", 1),
    np.dtype(np.int16): ("I", 2),
    np.dtype(np.int32): ("I", 4),
    np.dtype(np.int64): ("I", 8),
    np.dtype(np.uint8): ("U", 1),
    np.dtype(np.uint16): ("U", 2),
    np.dtype(np.uint32): ("U", 4),
    np.dtype(np.uint64): ("U", 8),
}


def parse_pcd_header(fp) -> Tuple[List[str], Dict[str, Any], bytes]:
    header_lines = []
    raw_header = b""
    while True:
        line = fp.readline()
        if not line:
            raise ValueError("Unexpected EOF while reading PCD header")
        raw_header += line
        s = line.decode("utf-8", errors="strict").strip()
        header_lines.append(s)
        if s.startswith("DATA"):
            break

    meta: Dict[str, Any] = {}
    meta["raw_lines"] = header_lines

    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0]
        vals = parts[1:]
        meta[key] = vals

    required_keys = ["FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "POINTS", "DATA"]
    for k in required_keys:
        if k not in meta:
            raise ValueError(f"Missing PCD header key: {k}")

    return header_lines, meta, raw_header


def pcd_build_dtype(meta: Dict[str, Any]) -> np.dtype:
    fields = meta["FIELDS"]
    sizes = list(map(int, meta["SIZE"]))
    types = meta["TYPE"]
    counts = list(map(int, meta["COUNT"]))

    dtype_fields = []
    for name, sz, ty, cnt in zip(fields, sizes, types, counts):
        base = PCD_TYPE_TO_DTYPE.get((ty, sz))
        if base is None:
            raise ValueError(f"Unsupported PCD field type/size: type={ty}, size={sz}, field={name}")
        if cnt == 1:
            dtype_fields.append((name, np.dtype(base)))
        else:
            dtype_fields.append((name, np.dtype(base), (cnt,)))
    return np.dtype(dtype_fields)


def read_pcd(path: str):
    with open(path, "rb") as fp:
        header_lines, meta, raw_header = parse_pcd_header(fp)
        dtype = pcd_build_dtype(meta)

        points = int(meta["POINTS"][0])
        data_mode = meta["DATA"][0].lower()

        if data_mode == "binary":
            data = fp.read()
            expected = points * dtype.itemsize
            if len(data) < expected:
                raise ValueError(f"PCD binary payload too short: got {len(data)}, expected {expected}")
            arr = np.frombuffer(data[:expected], dtype=dtype, count=points).copy()
        elif data_mode == "ascii":
            arr = np.genfromtxt(fp, dtype=dtype, max_rows=points)
            if arr.shape == ():
                arr = np.array([arr], dtype=dtype)
        else:
            raise ValueError(f"Unsupported PCD DATA mode: {data_mode}")

    return arr, header_lines, meta


def write_pcd(path: str, arr: np.ndarray, header_lines: List[str], meta: Dict[str, Any]):
    data_mode = meta["DATA"][0].lower()
    points = len(arr)

    # Rebuild header while preserving unknown/comment lines as much as possible
    new_lines = []
    for line in header_lines:
        if not line:
            new_lines.append(line)
            continue

        parts = line.split()
        key = parts[0] if parts else ""

        if key == "WIDTH":
            new_lines.append(f"WIDTH {points}")
        elif key == "HEIGHT":
            # preserve original HEIGHT unless invalid
            height = int(meta["HEIGHT"][0])
            if height != 1 and points % height == 0:
                new_lines.append(f"HEIGHT {height}")
            else:
                new_lines.append("HEIGHT 1")
        elif key == "POINTS":
            new_lines.append(f"POINTS {points}")
        else:
            new_lines.append(line)

    with open(path, "wb") as fp:
        header_blob = ("\n".join(new_lines) + "\n").encode("utf-8")
        fp.write(header_blob)

        if data_mode == "binary":
            fp.write(arr.tobytes(order="C"))
        elif data_mode == "ascii":
            # Build row formatter per field
            names = arr.dtype.names
            for row in arr:
                tokens = []
                for name in names:
                    value = row[name]
                    if np.isscalar(value):
                        tokens.append(str(value.item() if hasattr(value, "item") else value))
                    else:
                        tokens.extend(str(v.item() if hasattr(v, "item") else v) for v in np.ravel(value))
                fp.write((" ".join(tokens) + "\n").encode("utf-8"))
        else:
            raise ValueError(f"Unsupported PCD DATA mode: {data_mode}")


# ============================================================
# PLY
# ============================================================

PLY_SCALAR_MAP = {
    "char": np.int8,
    "uchar": np.uint8,
    "short": np.int16,
    "ushort": np.uint16,
    "int": np.int32,
    "uint": np.uint32,
    "float": np.float32,
    "double": np.float64,
}

PLY_DTYPE_TO_NAME = {np.dtype(v): k for k, v in PLY_SCALAR_MAP.items()}


def parse_ply_header(fp):
    header_lines = []
    first = fp.readline()
    if not first:
        raise ValueError("Empty file")
    if first.decode("utf-8", errors="strict").strip() != "ply":
        raise ValueError("Not a PLY file")
    header_lines.append("ply")

    fmt = None
    vertex_count = None
    vertex_props = []
    in_vertex = False

    while True:
        line = fp.readline()
        if not line:
            raise ValueError("Unexpected EOF while reading PLY header")
        s = line.decode("utf-8", errors="strict").rstrip("\n")
        header_lines.append(s)

        parts = s.split()
        if not parts:
            continue

        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element":
            in_vertex = (parts[1] == "vertex")
            if in_vertex:
                vertex_count = int(parts[2])
        elif parts[0] == "property" and in_vertex:
            if parts[1] == "list":
                raise ValueError("This script preserves vertex scalar properties only. "
                                 "List properties inside vertex are unsupported.")
            ptype = parts[1]
            pname = parts[2]
            if ptype not in PLY_SCALAR_MAP:
                raise ValueError(f"Unsupported PLY property type: {ptype}")
            vertex_props.append((pname, PLY_SCALAR_MAP[ptype], ptype))
        elif parts[0] == "end_header":
            break

    if fmt is None:
        raise ValueError("PLY format not found")
    if vertex_count is None:
        raise ValueError("PLY vertex element not found")

    dtype = np.dtype([(name, np.dtype(dt)) for name, dt, _ in vertex_props])

    return header_lines, fmt, vertex_count, vertex_props, dtype


def read_ply(path: str):
    with open(path, "rb") as fp:
        header_lines, fmt, vertex_count, vertex_props, dtype = parse_ply_header(fp)

        if fmt == "binary_little_endian":
            data = fp.read()
            expected = vertex_count * dtype.itemsize
            if len(data) < expected:
                raise ValueError(f"PLY binary payload too short: got {len(data)}, expected {expected}")
            arr = np.frombuffer(data[:expected], dtype=dtype, count=vertex_count).copy()
        elif fmt == "ascii":
            arr = np.genfromtxt(fp, dtype=dtype, max_rows=vertex_count)
            if arr.shape == ():
                arr = np.array([arr], dtype=dtype)
        else:
            raise ValueError(f"Unsupported PLY format: {fmt}")

    return arr, header_lines, fmt, vertex_props


def write_ply(path: str, arr: np.ndarray, header_lines: List[str], fmt: str, vertex_props):
    with open(path, "wb") as fp:
        fp.write(("ply\n").encode("utf-8"))

        wrote_format = False
        wrote_vertex = False
        in_vertex = False
        wrote_vertex_props = False

        for i, line in enumerate(header_lines[1:], start=1):
            parts = line.split()

            if not parts:
                fp.write((line + "\n").encode("utf-8"))
                continue

            key = parts[0]

            if key == "format":
                fp.write((f"format {fmt} 1.0\n").encode("utf-8"))
                wrote_format = True

            elif key == "element":
                elem_name = parts[1]
                if elem_name == "vertex":
                    fp.write((f"element vertex {len(arr)}\n").encode("utf-8"))
                    wrote_vertex = True
                    in_vertex = True
                    wrote_vertex_props = False
                else:
                    in_vertex = False
                    fp.write((line + "\n").encode("utf-8"))

            elif key == "property" and in_vertex:
                if not wrote_vertex_props:
                    for name, _, ptype_name in vertex_props:
                        fp.write((f"property {ptype_name} {name}\n").encode("utf-8"))
                    wrote_vertex_props = True
                # skip original vertex property lines
                continue

            elif key == "end_header":
                if in_vertex and not wrote_vertex_props:
                    for name, _, ptype_name in vertex_props:
                        fp.write((f"property {ptype_name} {name}\n").encode("utf-8"))
                fp.write(("end_header\n").encode("utf-8"))
                break

            else:
                fp.write((line + "\n").encode("utf-8"))

        if fmt == "binary_little_endian":
            fp.write(arr.tobytes(order="C"))
        elif fmt == "ascii":
            names = arr.dtype.names
            for row in arr:
                tokens = []
                for name in names:
                    value = row[name]
                    tokens.append(str(value.item() if hasattr(value, "item") else value))
                fp.write((" ".join(tokens) + "\n").encode("utf-8"))
        else:
            raise ValueError(f"Unsupported PLY format: {fmt}")


# ============================================================
# Main
# ============================================================

def infer_format(path: str):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".pcd":
        return "pcd"
    if ext == ".ply":
        return "ply"
    raise ValueError(f"Unsupported extension: {ext}")


def main():
    parser = argparse.ArgumentParser(description="Rotate/translate PCD or PLY point cloud map while preserving all fields.")
    parser.add_argument("--input", required=True, help="Input .pcd or .ply")
    parser.add_argument("--output", required=True, help="Output .pcd or .ply")
    parser.add_argument("--roll-deg", type=float, default=0.0, help="Roll in degrees")
    parser.add_argument("--pitch-deg", type=float, default=0.0, help="Pitch in degrees")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="Yaw in degrees")
    parser.add_argument("--tx", type=float, default=0.0, help="Translation X")
    parser.add_argument("--ty", type=float, default=0.0, help="Translation Y")
    parser.add_argument("--tz", type=float, default=0.0, help="Translation Z")

    args = parser.parse_args()

    in_fmt = infer_format(args.input)
    out_fmt = infer_format(args.output)
    if in_fmt != out_fmt:
        raise ValueError("Input and output extensions must match. "
                         "Use .pcd -> .pcd or .ply -> .ply")

    R, t = make_transform(
        args.roll_deg, args.pitch_deg, args.yaw_deg,
        args.tx, args.ty, args.tz
    )

    if in_fmt == "pcd":
        arr, header_lines, meta = read_pcd(args.input)
        apply_transform_to_xyz(arr, R, t)
        write_pcd(args.output, arr, header_lines, meta)

    elif in_fmt == "ply":
        arr, header_lines, fmt, vertex_props = read_ply(args.input)
        apply_transform_to_xyz(arr, R, t)
        write_ply(args.output, arr, header_lines, fmt, vertex_props)

    else:
        raise RuntimeError("Unreachable")

    print(f"[OK] Saved: {args.output}")
    print(f"     roll={args.roll_deg} deg, pitch={args.pitch_deg} deg, yaw={args.yaw_deg} deg")
    print(f"     tx={args.tx}, ty={args.ty}, tz={args.tz}")
    print(f"     points={len(arr)}")


if __name__ == "__main__":
    main()
