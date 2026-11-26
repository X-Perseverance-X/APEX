"""Convert the supplied MARK_2_LEG assembly and STL exports to link-local visuals.

Run with the source CAD directory as the sole argument. Only generated meshes
are written. The input documents are read without opening their native features.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path

import numpy as np

ASSETS = Path(__file__).resolve().parents[1] / "src/apex_rl/assets/mark2_leg"
STL_TRIANGLE = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("tag", "<u2")])
NIBBLE_SWAP = bytes((byte % 16) * 16 + byte // 16 for byte in range(256))

# The three Coxa_head instances are the horns at the three joint axes.
# Bodies are grouped by which hinge moves them, including the visible mirrored
# brackets in the source assembly. The hidden originals are omitted.
PARTS = {
    "mount": ("Coxa_Body-1",),
    "coxa": ("Coxa_head-1", "AynalamaFemur_Holder-2"),
    "femur": (
        "Coxa_head-2",
        "Coxa_head-3",
        "Tibia_Holder_1-1",
        "AynalamaTibia_Holder_2-2",
        "Tibia_Holder_3-1",
    ),
    "tibia": (
        "Tibia_1-1",
        "Tibia_Head-1",
        "Tibia_Support_1-1",
        "Tibia_Support_2-1",
        "Tibia_Support_3-1",
    ),
}


def read_instance_tree(data: bytes) -> ET.Element:
    """Read the assembly's compressed component instance XML."""
    for record in re.finditer(b"\x14\x00\x06\x00\x08\x00", data):
        offset = record.start()
        _, size, expanded_size, length, extra = struct.unpack_from("<IIIHH", data, offset + 10)
        start = offset + 26
        if data[start : start + length].translate(NIBBLE_SWAP) != b"swXmlContents/COMPINSTANCETREE":
            continue
        start += length + extra
        xml = zlib.decompress(data[start : start + size], wbits=-15)
        if len(xml) != expanded_size:
            raise ValueError("SolidWorks instance tree size mismatch")
        return ET.fromstring(xml)
    raise ValueError("SolidWorks component instance tree missing")


def read_stl(data: bytes) -> np.ndarray:
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * STL_TRIANGLE.itemsize or count == 0:
        raise ValueError("Expected a non-empty binary STL")
    triangles = np.frombuffer(data, dtype=STL_TRIANGLE, offset=84)["vertices"].astype(np.float64)
    if not np.isfinite(triangles).all():
        raise ValueError("STL contains non-finite vertices")
    return triangles * 0.001


def write_stl(path: Path, vertices: np.ndarray) -> None:
    edges = vertices[:, 1:] - vertices[:, :1]
    normal = np.cross(edges[:, 0], edges[:, 1])
    magnitude = np.linalg.norm(normal, axis=1)
    valid = magnitude > 1e-12
    mesh = np.zeros(int(valid.sum()), dtype=STL_TRIANGLE)
    mesh["normal"] = normal[valid] / magnitude[valid, None]
    mesh["vertices"] = vertices[valid]
    header = b"META-1 / MARK_2_LEG / metres".ljust(80) + struct.pack("<I", len(mesh))
    path.write_bytes(header + mesh.tobytes())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    assembly = args.source / "MARK_2_LEG.SLDASM"
    stl_files = {
        file.stem.casefold(): file
        for file in args.source.iterdir()
        if file.is_file() and file.suffix.casefold() == ".stl"
    }
    data = assembly.read_bytes()
    xml = read_instance_tree(data)
    models = {model.get("id"): model for model in xml.findall(".//{*}swModel")}
    transforms = {}
    meshes = {}
    hashes = {assembly.name: hashlib.sha256(data).hexdigest()}
    for ref in xml.findall(".//{*}swReference"):
        if ref.get("swHidden") == "YES" or ref.get("swSuppressed") == "YES":
            continue
        part = ref.attrib["swName"]
        name = f"{part}-{ref.attrib['swReferenceNumber']}"
        mirrored = part.startswith("Aynalama")
        file = stl_files[part.removeprefix("Aynalama").casefold()]
        data = file.read_bytes()
        hashes[file.name] = hashlib.sha256(data).hexdigest()
        triangles = read_stl(data)
        if mirrored:
            triangles[:, :, 2] *= -1
            triangles = triangles[:, [0, 2, 1]]
        bbox = np.fromstring(models[ref.attrib["swModelRef"]].attrib["swBoundingBox"], sep=" ")
        bounds = triangles.reshape(-1, 3)
        if not np.allclose(np.ptp(bounds, axis=0), bbox[3:] - bbox[:3], atol=0.00006, rtol=0):
            raise ValueError(f"{name}: STL size does not match the referenced CAD part")
        # SolidWorks exports these individual STLs with their minimum corner
        # shifted to the origin. Recover their native origins from the AABB.
        triangles += bbox[:3] - bounds.min(axis=0)
        matrix = np.fromstring(ref.attrib["swTransform"], sep=" ").reshape(4, 4)
        rotation, position = matrix[:3, :3], matrix[3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8):
            raise ValueError(f"{name}: component transform is not rigid")
        transforms[name] = (rotation, position)
        meshes[name] = triangles @ rotation + position

    # The main axis emerges through the 15 mm counterbore in the horn. Its
    # centre is 12 mm from the left edge of the 62 mm STL, i.e. X=-19 mm in CAD.
    horn_centres = []
    for index in (1, 2, 3):
        rotation, position = transforms[f"Coxa_head-{index}"]
        horn_centres.append(np.array([-0.019, 0.0, 0.0]) @ rotation + position)
    outward = transforms["Coxa_head-1"][0][0]
    up = np.array([0.0, 1.0, 0.0])
    across = np.cross(up, outward)
    frame = np.column_stack([outward, across, up])
    hip, femur, knee = np.array(horn_centres) @ frame
    # Intersect the parallel femur/knee hinge axes with the coxa centre plane.
    hip[2] = femur[2]
    femur[1] = knee[1] = hip[1]
    foot = transforms["Tibia_Support_3-1"][1] @ frame
    foot[1] = hip[1]
    pivots = {"mount": hip, "coxa": hip, "femur": femur, "tibia": knee}
    tips = {"mount": femur, "coxa": femur, "femur": knee, "tibia": foot}
    ASSETS.mkdir(parents=True, exist_ok=True)
    lengths = {}
    for link, parts in PARTS.items():
        vector = tips[link] - pivots[link]
        pitch = float(np.arctan2(vector[2], vector[0]))
        c, s = np.cos(pitch), np.sin(pitch)
        unpitch = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
        original = np.concatenate(
            [(meshes[name] @ frame - pivots[link]) @ unpitch for name in parts]
        )
        write_stl(ASSETS / f"{link}_right.stl", original)
        mirrored = original[:, [0, 2, 1]].copy()
        mirrored[:, :, 1] *= -1
        write_stl(ASSETS / f"{link}_left.stl", mirrored)
        lengths[link] = round(float(np.linalg.norm(vector)) * 1000, 3)
    manifest = {
        "source": assembly.name,
        "source_sha256": hashes,
        "assembled_instances": PARTS,
        "cad_pivot_distances_mm": lengths,
        "units": "metres",
        "mirror": "local Y",
    }
    (ASSETS / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(lengths))


if __name__ == "__main__":
    main()
