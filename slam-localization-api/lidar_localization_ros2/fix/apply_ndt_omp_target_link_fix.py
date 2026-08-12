#!/usr/bin/env python3
"""Replace legacy ndt_omp_ros2 CMake variables without depending on line numbers."""

from pathlib import Path
import re
import sys


LEGACY_INCLUDE = "${ndt_omp_ros2_INCLUDE_DIRS}"
LEGACY_LIBRARY = "${ndt_omp_ros2_LIBRARIES}"
IMPORTED_TARGET = "ndt_omp_ros2::ndt_omp"


def update_command(
    text: str, command: str, target: str, replacement: tuple[str, str]
) -> str:
    match = re.search(
        rf"\b{re.escape(command)}\s*\(\s*{re.escape(target)}\b", text
    )
    if match is None:
        return text

    start = match.start()
    cursor = match.end()
    depth = 1
    while cursor < len(text) and depth:
        if text[cursor] == "(":
            depth += 1
        elif text[cursor] == ")":
            depth -= 1
        cursor += 1

    if depth:
        raise RuntimeError(f"unterminated {command}() command")

    block = text[start:cursor].replace(*replacement)
    return text[:start] + block + text[cursor:]


def update_cmake(path: Path) -> bool:
    original = path.read_text(encoding="utf-8")
    updated = update_command(
        original,
        "target_include_directories",
        "g2_ndt_score",
        (LEGACY_INCLUDE, ""),
    )
    updated = update_command(
        updated,
        "target_link_libraries",
        "g2_ndt_score",
        (LEGACY_LIBRARY, IMPORTED_TARGET),
    )

    if updated == original:
        print("ndt_omp target-link fix is not needed for this upstream revision")
        return False

    path.write_text(updated, encoding="utf-8")
    print("replaced legacy ndt_omp_ros2 CMake variables with the imported target")
    return True


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} CMakeLists.txt", file=sys.stderr)
        return 2

    path = Path(sys.argv[1])
    if not path.is_file():
        print(f"CMake file not found: {path}", file=sys.stderr)
        return 2

    update_cmake(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
