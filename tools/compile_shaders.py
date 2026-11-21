#!/usr/bin/env python3
"""Compile GLSL shaders from shaders/src into shaders/compiled.

For every shaders/src/<name>.<stage> (stage = vert | frag | comp) produces:
  <name>.<stage>.spv  - SPIR-V for the Vulkan backend (Windows / Linux)
  <name>.<stage>.msl  - Metal Shading Language for the Metal backend (macOS)

Requires glslang (glslangValidator / glslang) - from the Vulkan SDK (https://vulkan.lunarg.com/),
on PATH, or unpacked from a glslang release (https://github.com/KhronosGroup/glslang/releases)
into .tools/glslang/ in the repository. spirv-cross (Vulkan SDK) is needed for the Metal (.msl)
outputs; without it only SPIR-V is rebuilt and the .msl files are left as they are.
Run from anywhere: python3 tools/compile_shaders.py

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


def find_glslang() -> str | None:
    for name in ("glslangValidator", "glslang"):
        if found := shutil.which(name):
            return found
    for name in ("glslangValidator", "glslang"):
        for suffix in ("", ".exe"):
            local = ROOT / ".tools" / "glslang" / "bin" / f"{name}{suffix}"
            if local.is_file():
                return str(local)
    return None


def main() -> None:
    glslang = find_glslang()
    if glslang is None:
        sys.exit("glslang not found - install the Vulkan SDK or unpack a glslang release into .tools/glslang")
    spirv_cross = shutil.which("spirv-cross")
    if spirv_cross is None:
        print("warning: spirv-cross not found - Metal (.msl) shaders are NOT updated")

    OUT.mkdir(parents=True, exist_ok=True)
    sources = sorted(p for p in SRC.iterdir() if p.suffix.lstrip(".") in STAGES)
    for src in sources:
        spv = OUT / f"{src.name}.spv"
        msl = OUT / f"{src.name}.msl"
        run([glslang, "-V", "--target-env", "vulkan1.1", str(src), "-o", str(spv)])
        if spirv_cross:
            run([spirv_cross, str(spv), "--msl", "--msl-version", "20100", "--output", str(msl)])
            print(f"  {src.name} -> {spv.name}, {msl.name}")
        else:
            print(f"  {src.name} -> {spv.name}")
    print(f"compiled {len(sources)} shader(s)")


if __name__ == "__main__":
    main()
