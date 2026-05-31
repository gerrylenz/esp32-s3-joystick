# Copyright 2025 Espressif Systems (Shanghai) PTE LTD.
# Adapted from: https://github.com/espressif/idf-eclipse-plugin/tree/master/resources/fix_compile_commands

"""Remove GCC-only flags from compile_commands.json for clangd."""

import json
import os
import re

# Flags clang/clangd does not understand; keep other -m/-f flags (e.g. -fno-rtti, -std glue).
GCC_ONLY_FLAGS = {
    "-fno-tree-switch-conversion",
    "-fno-shrink-wrap",
    "-fstrict-volatile-bitfields",
    "-mtext-section-literals",
    "-mlongcalls",
}


def find_compile_commands_json(start_dir):
    for root, _dirs, files in os.walk(start_dir):
        if "compile_commands.json" in files:
            return os.path.join(root, "compile_commands.json")
    return None


def filter_args(args):
    filtered = [
        arg
        for arg in args
        if arg not in GCC_ONLY_FLAGS and not re.match(r"^-march=.*", arg)
    ]
    # ESP-IDF newlib reent.h triggers -Wvisibility under clang/clangd with
    # -fvisibility=hidden from the toolchain; GCC builds are unaffected.
    if "-Wno-visibility" not in filtered:
        filtered.append("-Wno-visibility")
    return filtered


def remove_unsupported_flags(compile_commands):
    for entry in compile_commands:
        args = entry.get("arguments") or entry.get("command", "").split()
        filtered_args = filter_args(args)

        if "arguments" in entry:
            entry["arguments"] = filtered_args
        elif "command" in entry:
            entry["command"] = " ".join(filtered_args)
    return compile_commands


def main():
    build_dir = os.path.join(os.path.dirname(__file__), "build")
    cc_path = find_compile_commands_json(build_dir)

    if not cc_path:
        print("compile_commands.json not found.")
        return

    with open(cc_path, "r", encoding="utf-8") as f:
        compile_commands = json.load(f)

    cleaned_commands = remove_unsupported_flags(compile_commands)

    with open(cc_path, "w", encoding="utf-8") as f:
        json.dump(cleaned_commands, f, indent=2)

    print(f"Removed GCC-only flags from: {cc_path}")


if __name__ == "__main__":
    main()
