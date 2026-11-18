#!/usr/bin/env python3
"""Compile GLSL shaders from shaders/src into shaders/compiled.

For every shaders/src/<name>.<stage> (stage = vert | frag | comp) produces:
  <name>.<stage>.spv  - SPIR-V for the Vulkan backend (Windows / Linux)
  <name>.<stage>.msl  - Metal Shading Language for the Metal backend (macOS)

Requires glslangValidator and spirv-cross on PATH (both ship with the Vulkan SDK:
https://vulkan.lunarg.com/). Run from anywhere: python3 tools/compile_shaders.py

SDL_GPU resource binding rules (keep them in mind when adding uniforms/textures):
https://wiki.libsdl.org/SDL3/SDL_CreateGPUShader
"""

import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "shaders" / "src"
OUT = ROOT / "shaders" / "compiled"
STAGES = {"vert", "frag", "comp"}


def run(cmd: list[str]) -> None:
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(result.stdout + result.stderr)
        sys.exit(f"failed: {' '.join(cmd)}")


def main() -> None:
    for tool in ("glslangValidator", "spirv-cross"):
        if shutil.which(tool) is None:
            sys.exit(f"{tool} not found on PATH - install the Vulkan SDK")

    OUT.mkdir(parents=True, exist_ok=True)
    sources = sorted(p for p in SRC.iterdir() if p.suffix.lstrip(".") in STAGES)
    for src in sources:
        spv = OUT / f"{src.name}.spv"
        msl = OUT / f"{src.name}.msl"
        run(["glslangValidator", "-V", "--target-env", "vulkan1.1", str(src), "-o", str(spv)])
        run(["spirv-cross", str(spv), "--msl", "--msl-version", "20100", "--output", str(msl)])
        print(f"  {src.name} -> {spv.name}, {msl.name}")
    print(f"compiled {len(sources)} shader(s)")


if __name__ == "__main__":
    main()
