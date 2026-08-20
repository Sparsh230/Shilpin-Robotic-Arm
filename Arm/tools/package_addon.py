"""
package_addon.py -- build robot_arm_twin.zip for Blender's add-on installer.

    python tools/package_addon.py

The zip contains the package folder at its root, which is the layout
"Install from Disk..." expects.  __pycache__ and any vendored pyserial are
left out so the archive stays clean.
"""

import os
import sys
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "blender_addon", "robot_arm_twin")
OUT = os.path.join(ROOT, "blender_addon", "robot_arm_twin.zip")

SKIP_DIRS = {"__pycache__", "_vendor"}


def main():
    if not os.path.isdir(SRC):
        print("source not found: %s" % SRC)
        return 1
    count = 0
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
        for folder, dirs, files in os.walk(SRC):
            dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
            for name in sorted(files):
                if name.endswith((".pyc", ".pyo")):
                    continue
                full = os.path.join(folder, name)
                rel = os.path.relpath(full, os.path.dirname(SRC))
                z.write(full, rel.replace(os.sep, "/"))
                count += 1
    print("wrote %s (%d files, %.1f KB)"
          % (OUT, count, os.path.getsize(OUT) / 1024.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
