"""Launch native MuJoCo under uv; resolve macOS libpython via the base runtime."""

import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path


def main():
    args = sys.argv[1:]
    module = "phase1b.scripts.view_scene"
    if args[:1] == ["--human"]:
        module, args = "phase1b.human", args[1:]
    env = os.environ.copy()
    if platform.system() == "Darwin":
        executable = shutil.which("mjpython")
        if not executable:
            raise RuntimeError("mjpython missing; run uv sync --locked")
        paths = [str(Path(sys.base_prefix) / "lib")]
        if env.get("DYLD_FALLBACK_LIBRARY_PATH"):
            paths.append(env["DYLD_FALLBACK_LIBRARY_PATH"])
        env["DYLD_FALLBACK_LIBRARY_PATH"] = ":".join(paths)
    else:
        executable = sys.executable
    raise SystemExit(
        subprocess.call(
            [executable, "-m", module, *args], env=env
        )
    )


if __name__ == "__main__":
    main()
