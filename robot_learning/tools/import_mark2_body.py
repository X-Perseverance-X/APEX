"""Convert the user-supplied chassis STL exports from millimetres to metres."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from import_mark2_leg import read_stl, write_stl

ASSETS = Path(__file__).resolve().parents[1] / "src/apex_rl/assets"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    layout = json.loads((ASSETS / "body_layout.json").read_text(encoding="utf-8"))
    stls = {
        source.stem.casefold(): source
        for source in args.source.iterdir()
        if source.is_file() and source.suffix.casefold() == ".stl"
    }
    output = ASSETS / "mark2_body"
    output.mkdir(exist_ok=True)
    sources = {}
    for mesh in sorted({p["mesh"] for p in layout["parts"]}):
        source = stls[mesh.casefold()]
        raw = source.read_bytes()
        vertices = read_stl(raw)
        write_stl(output / f"{mesh}.stl", vertices)
        sources[source.name] = {
            "source_sha256": hashlib.sha256(raw).hexdigest(),
            "dimensions_mm": (np.ptp(vertices.reshape(-1, 3), axis=0) * 1000).round(3).tolist(),
        }
    (output / "manifest.json").write_text(
        json.dumps({"units": "metres", "sources": sources}, indent=2) + "\n", encoding="utf-8"
    )
    print(f"Converted {len(sources)} chassis and camera meshes")


if __name__ == "__main__":
    main()
