#!/usr/bin/env python3

import sys
from pathlib import Path
import os
import re

def get_header(input: str) -> str | None:
    path = Path(input)
    if (not path.is_dir()):
        return None

    properties = path / "library.properties"

    if properties.is_file():
        with open(properties, "r") as file:
            for line in file:
                line = line.strip()
                if len(line) == 0 or line.startswith("!") or line.startswith("#"):
                    continue

                if not line.startswith("includes"):
                    continue

                header = line.removeprefix("includes=")

                if not header.endswith("hpp") and not header.endswith("h"):
                    continue

                return header

    candidates = [Path(f"{path.name}.h"), Path(f"{path.name}.hpp"), Path(f"src/{path.name}.h"), Path(f"src/{path.name}.hpp")]

    for candidate in candidates:
        candidate_path = path / candidate

        if candidate_path.is_file():
            return candidate_path.name

    # print(f"failed to find for {input}", file=sys.stderr)
    return None

target_directory = sys.argv[1]
headers = [(re.compile(rf"#include\s+<{header}>"), Path(input).name) for input in sys.argv[2].split(";") if (header := get_header(input)) is not None and input != target_directory]

deps = set()

for root, dirs, files in os.walk(target_directory):
    root = Path(root)
    for file in files:
        file = root / file

        if file.suffix not in [ ".c", ".h", ".cpp", ".hpp" ]:
            continue

        if "example" in str(file):
            continue

        if not file.is_file():
            continue

        with open(file, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                for regex, pkg in headers:
                    if regex.search(line):
                        deps.add(pkg)

print(";".join(deps))
