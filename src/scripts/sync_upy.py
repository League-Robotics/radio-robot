#!/usr/bin/env python3
"""sync_upy.py -- vendor the kernel + Nezha leaf into the nezha-upy repo.

radio-robot is the single SOURCE of the DiffDrive kernel
(src/firm/diffdrive/, guarded by src/tests/diffdrive/) and the Nezha
motor leaf. The MicroPython-first rebuild
(https://github.com/League-Robotics/nezha-upy — see
clasi/issues/micropython-first-rebuild.md) consumes them VENDORED, never
edited there. This script is the sync (sibling of sync_pxt.py, which
does the same for the MakeCode extension):

    uv run python src/scripts/sync_upy.py [path-to-nezha-upy-checkout]

Copies, verbatim (full comments kept -- unlike the MakeCode branch there
is no size ceiling):

    src/firm/diffdrive/differential_drive.{h,cpp}  -> vendor/
    src/firm/hardware/nezha/nezha_motor.{h,cpp}    -> vendor/
    src/firm/hardware/generic/motor_armor.h        -> vendor/
    src/tests/fixtures/wire_golden_vectors.txt     -> tests/fixtures/

The nezha_motor files depend on firmware HAL headers; the nezha-upy
native layer either ports them (M1's plan) or builds them against its
own minimal HAL shims -- either way the vendored copy is the reference
and the sync-diff is the drift gate. Run after any kernel/leaf/schema
change and commit in nezha-upy.
"""
import pathlib
import shutil
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

_COPIES = [
    ("src/firm/diffdrive/differential_drive.h", "vendor/differential_drive.h"),
    ("src/firm/diffdrive/differential_drive.cpp", "vendor/differential_drive.cpp"),
    ("src/firm/hardware/nezha/nezha_motor.h", "vendor/nezha_motor.h"),
    ("src/firm/hardware/nezha/nezha_motor.cpp", "vendor/nezha_motor.cpp"),
    ("src/firm/hardware/generic/motor_armor.h", "vendor/motor_armor.h"),
    ("src/tests/fixtures/wire_golden_vectors.txt",
     "tests/fixtures/wire_golden_vectors.txt"),
]


def main() -> int:
    dest = pathlib.Path(sys.argv[1] if len(sys.argv) > 1
                        else _REPO_ROOT.parent / "nezha-upy")
    if not (dest / ".git").exists():
        print(f"not a git checkout: {dest}")
        return 1
    for src_rel, dest_rel in _COPIES:
        src = _REPO_ROOT / src_rel
        target = dest / dest_rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, target)
        print(f"  {dest_rel}: {target.stat().st_size} bytes")
    print(f"synced into {dest} -- review and commit there")
    return 0


if __name__ == "__main__":
    sys.exit(main())
