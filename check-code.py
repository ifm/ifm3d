#!/usr/bin/python3

# SPDX-License-Identifier: Apache-2.0
# Copyright (C) ifm electronic gmbh
#
# THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.

# Provides functions to format C/C++ files using clang-format
import argparse
from collections.abc import Generator
import functools
import json
import multiprocessing
import os
import re
import shutil
import subprocess
import sys
from typing import Any

assert sys.version_info > (3, 5), "python 3.5 or more required"

MAX_PARALLEL_PROCESSES = 10

SUPPORTED_TOOL_VERSIONS = {
    "clang_format": 14,
    "clang_tidy": 20,
    "cppcheck": 2,
}


# Extension and folders to be excludes from formatting
APPLY_EXTENSIONS = (".cxx", ".cpp", ".c", ".hxx", ".hh", ".cc", ".hpp", ".h")
EXCLUDES = list([re.compile(p) for p in [
    r".*/_deps/.*"
]])

VALID_ROOTS = [os.path.abspath(os.getcwd())]

def run_clang_format(file: str, args: dict[str, Any]) -> bool:
    cmd = [args["clang_format"], "-i", "-style=file"]

    if not args["fix"]:
        cmd.append("--dry-run")
        cmd.append("--Werror")

    cmd.append(file)

    return subprocess.call(cmd) == 0


def run_clang_tidy(file: str, args: dict[str, Any]) -> bool:
    cmd = [args["clang_tidy"]]

    if "config_file" in args:
        cmd.append(f"--config-file={args['config_file']}")

    if "build_dir" in args:
        cmd.append(f"-p={args['build_dir']}")
        cmd.append(f"--exclude-header-filter=^{args['build_dir']}/_deps/.*")

    if "fix" in args and args["fix"]:
        cmd.append("--fix")
        cmd.append("--fix-errors")

    cmd.append("--quiet")
    cmd.append("--use-color")
    cmd.append(file)

    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    if p.returncode != 0:
        print(p.stdout.decode("utf-8"))

    return p.returncode == 0

def run_cppcheck(args: dict[str, Any]) -> bool:
    cmd = ["cppcheck"]

    if "build_dir" in args:
        cmd.append(f"--project={args['build_dir']}/compile_commands.json")
        cmd.append(f"--cppcheck-build-dir={args['build_dir']}/cppcheck")
        cmd.append(f"--suppress=*:{args['build_dir']}/*")
        cmd.append(f"--output-file={args['build_dir']}/cppcheck.xml")
        os.makedirs(os.path.join(args["build_dir"], "cppcheck"), exist_ok=True)

    cmd.append(f"--suppress=*:/usr/*")

    cmd.extend([
        "--std=c++17",
        "--xml",
        "--error-exitcode=1",
        "-j", 
        str(MAX_PARALLEL_PROCESSES),
        f"-D{"_WIN32" if sys.platform == "nt" else "__unix__"}",
    ])

    return subprocess.call(cmd) == 0


def include_file(file: str) -> bool:
    if not any([os.path.abspath(file).startswith(root) for root in VALID_ROOTS]):
        return False

    # check for file extension
    if not file.endswith(APPLY_EXTENSIONS):
        return False

    # Check for excludes
    if any([exclude.match(file) for exclude in EXCLUDES]):
        return False
    
    return True

def glob_files(path: str) -> list[str]:
    filelist = set()

    for root, _, files in os.walk(os.path.relpath(path, os.getcwd())):
        for file in files:
            path = os.path.join(root, file)

            if include_file(file):
                filelist.add(path)

    return list(filelist)


def get_compile_db_files(compile_db: str) -> list[str]:
    with open(compile_db, "r") as f:
        compile_db = json.loads(f.read())

    filelist = set()
    for entry in compile_db:
        file_path: str = entry["file"]
        rel_path: str = os.path.relpath(file_path, os.getcwd())

        if include_file(rel_path):
            filelist.add(rel_path)

    return list(filelist)

def filter_files(files: list[str], files_to_check: list[str]) -> Generator[str, None, None]:
    if not files_to_check:
        for f in files:
            yield f

    for f1 in files_to_check:
        for f2 in files:
            if os.path.abspath(f1) == os.path.abspath(f2):
                yield f2
                break 

def process_files(files: list[str], func: str, args: dict[str, Any]) -> bool:
    with multiprocessing.Pool(MAX_PARALLEL_PROCESSES) as p:
        try:
            ok = True
            for i, result in enumerate(
                p.imap(functools.partial(globals()[func], args=args), files)
            ):
                ok = ok and result

                print(f"[{i+1}/{len(files)}] {files[i]}: {'\x1b[6;32mOK\x1b[0m' if result else '\x1b[6;31mFAIL\x1b[0m'}", flush=True, file=sys.stderr)
            
            return ok
        except Exception as e:
            print("Error while processing files", e, flush=True, file=sys.stderr)

    return False


def check_clang_version(tool_path: str, supported_version: int) -> None:
    """
    Function checks the version of a tool
    """
    try:
        if not shutil.which(tool_path):
            print("Unable to find tool executable")

        ret = os.popen("{} --version".format(tool_path)).read()
        match = re.search(
            r"(?P<application>.*?) version (?P<version_major>[0-9]+)(?:\.(?P<version_minor>[0-9]+))?(?:\.(?P<version_patch>[0-9]+))?(?P<suffix>.*)",
            ret,
        )

        if not match:
            print("Unable to check tool version")
            exit(1)

        version_major = int(match.group("version_major"))

        if version_major != supported_version:
            print(
                f"Unsupported tool version: {version_major} expected {supported_version}"
            )
            exit(1)

    except Exception as e:
        print("Error while checking tool version", e)
        exit(1)

def check_cppcheck_version(tool_path: str, supported_version: int) -> None:
    """
    Function checks the version of cppcheck
    """
    try:
        if not shutil.which(tool_path):
            print("Unable to find tool executable")

        ret = os.popen("{} --version".format(tool_path)).read()
        match = re.search(
            r"Cppcheck (?P<version_major>[0-9]+)(?:\.(?P<version_minor>[0-9]+))?(?:\.(?P<version_patch>[0-9]+))?(?P<suffix>.*)",
            ret,
        )

        if not match:
            print("Unable to check tool version")
            exit(1)

        version_major = int(match.group("version_major"))

        if version_major != supported_version:
            print(
                f"Unsupported tool version: {version_major} expected {supported_version}"
            )
            exit(1)

    except Exception as e:
        print("Error while checking tool version", e)
        exit(1)

def check_compile_db(build_dir) -> bool:
    compile_db = os.path.join(build_dir, "compile_commands.json")
    if not os.path.exists(compile_db):
        print(f"Unable to find compile_commands.json in {build_dir}. Please make sure to run cmake before calling this script.")
        exit(1)

def command_clang_tidy(args: dict[str, Any]) -> bool:
    check_clang_version(args["clang_tidy"], SUPPORTED_TOOL_VERSIONS["clang_tidy"])
    check_compile_db(args["build_dir"])
    files = get_compile_db_files(
        os.path.join(args["build_dir"], "compile_commands.json")
    )
    files = list(filter_files(files, [x[0] for x in args["file"]]))
    return process_files(files, "run_clang_tidy", args)

def command_clang_format(args: dict[str, Any]) -> bool:
    check_clang_version(
        args["clang_format"], SUPPORTED_TOOL_VERSIONS["clang_format"]
    )
    files = glob_files(args["path"])
    files = list(filter_files(files, [x[0] for x in args["file"]]))
    return process_files(files, "run_clang_format", args)

def command_cppcheck(args: dict[str, Any]) -> bool:
    check_cppcheck_version(args["cppcheck"], SUPPORTED_TOOL_VERSIONS["cppcheck"])
    check_compile_db(args["build_dir"])
    return run_cppcheck(args)

# entry point
if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    cmd_clang_tidy = subparsers.add_parser(
        "clang-tidy", help="Analyze the code using clang-tidy"
    )
    cmd_clang_tidy.add_argument(
        "--clang-tidy", help="Path to clang-tidy executable", default=f"clang-tidy-{SUPPORTED_TOOL_VERSIONS['clang_tidy']}"
    )
    cmd_clang_tidy.add_argument(
        "--config-file",
        help="Path to the clang-tidy configuration file",
        default=".clang-tidy",
    )
    cmd_clang_tidy.add_argument(
        "--build-dir", help="Path to the build directory", default="build"
    )
    cmd_clang_tidy.add_argument(
        "--fix", help="Apply fixes to the code", action="store_true"
    )

    cmd_cppcheck = subparsers.add_parser(
        "cppcheck", help="Analyze the code using cppcheck"
    )
    cmd_cppcheck.add_argument(
        "--cppcheck", help="Path to cppcheck executable", default="cppcheck"
    )
    cmd_cppcheck.add_argument(
        "--build-dir", help="Path to the build directory", default="build"
    )
    cmd_clang_tidy.add_argument(
        "--file", 
        help="Path to a single file to check, can be repeated for multiple files", 
        default=[], 
        action="append", 
        nargs='*'
    )

    cmd_clang_format = subparsers.add_parser(
        "clang-format", help="Check formatting using clang-format"
    )
    cmd_clang_format.add_argument(
        "--clang-format", help="Path to clang-format executable", default=f"clang-format-{SUPPORTED_TOOL_VERSIONS['clang_format']}"
    )
    cmd_clang_format.add_argument("--path", help="Path to the source code", default="modules")
    cmd_clang_format.add_argument(
        "--fix", help="Apply formatting to the code", action="store_true"
    )
    cmd_clang_format.add_argument(
        "--file", 
        help="Path to a single file to check, can be repeated for multiple files", 
        default=[], 
        action="append", 
        nargs='*'
    )

    args = vars(parser.parse_args())

    if "build_dir" in args:
        VALID_ROOTS.append(os.path.abspath(args["build_dir"]))

    if args["command"] is None:
        parser.print_help()
        quit(0)

    cmd = f"command_{args["command"].replace("-", "_")}"
    if cmd not in globals():
        print(f"Unknown command: {args['command']}")
        parser.print_help()
        quit(1)

    ok = globals()[f"command_{args["command"].replace("-", "_")}"](args)

    if ok:
        print('\x1b[6;32m' + 'All OK!' + '\x1b[0m')
    else:
        print('\x1b[6;31m' + 'FAILED!' + '\x1b[0m')

    quit(0 if ok else 1)

