#!/usr/bin/env python3
"""sync_pxt.py -- vendor the DiffDrive kernel into the MakeCode branch.

The MakeCode extension lives on the orphan branch `makecode` (checked
out as a worktree, conventionally ../radio-robot-makecode). Its
diffdrive.{h,cpp} are VENDORED copies of src/firm/diffdrive/
differential_drive.{h,cpp} with full-line comments stripped (MakeCode
extensions carry a 64 KB size guidance and the kernel is comment-heavy;
the stripped pair measures ~45 KB against ~70 KB unstripped).

Run from the repo root on master after any kernel change:

    uv run python src/scripts/sync_pxt.py [path-to-makecode-worktree]

then commit on the makecode branch. The provenance header (the leading
comment block of each file) is kept; only interior full-line comments
are dropped. The include is rewritten to the branch's flat layout.
"""
import pathlib
import re
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_PACKAGE = _REPO_ROOT / "src" / "firm" / "diffdrive"


def strip(text: str) -> str:
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    header = True
    for line in lines:
        s = line.lstrip()
        if header and s.startswith("//"):
            out.append(line)  # keep the provenance header block
            continue
        header = False
        if s.startswith("//"):
            continue  # drop interior full-line comments
        out.append(line)
    return re.sub(r"\n{3,}", "\n\n", "".join(out))


def main() -> int:
    dest = pathlib.Path(sys.argv[1] if len(sys.argv) > 1
                        else _REPO_ROOT.parent / "radio-robot-makecode")
    if not (dest / "pxt.json").is_file():
        print(f"not a makecode extension checkout: {dest}")
        return 1
    header = strip((_PACKAGE / "differential_drive.h").read_text())
    source = strip((_PACKAGE / "differential_drive.cpp").read_text())
    source = source.replace('#include "differential_drive.h"',
                            '#include "diffdrive.h"')
    (dest / "diffdrive.h").write_text(header)
    (dest / "diffdrive.cpp").write_text(source)
    for name in ("diffdrive.h", "diffdrive.cpp"):
        size = (dest / name).stat().st_size
        print(f"  {name}: {size} bytes")
    print(f"synced into {dest} -- review and commit on the makecode branch")
    return 0


if __name__ == "__main__":
    sys.exit(main())
