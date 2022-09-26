#!/usr/bin/python3

# SPDX-License-Identifier: Apache-2.0
# Copyright (C) ifm electronic gmbh
#
# THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.

# Provides functions to format C/C++ files using clang-format
import shutil
import re
import subprocess
import os
import multiprocessing
import functools
import sys
assert sys.version_info > (3, 5), "python 3.5 or more required"

max_parallel_process = 10

# The bare Minimum clang-format version
minimum_required_clang_format_version = "10.0.0"


# Extension and folders to be excludes from formatting
apply_extensions = (".cxx", ".cpp", ".c", ".hxx", ".hh", ".cc", ".hpp", ".h")
include_folders = ["modules", "examples"]
exclude_folders = ["modules/device/include/ifm3d/contrib"]


def format_file(clangf_exe, file, dry_run=False):
    args = [clangf_exe, "-i", "-style=file"]

    if dry_run:
        args.append("--dry-run")
        args.append("--Werror")

    return subprocess.call([*args, file]) == 0


def clang_format(clangf_exe, dry_run=False):
    """
    Function runs the clang formatter on the files
    with extension mention in  'apply_extensions' and
    in the folders mentioned in 'include_folders'

    Return:
        (bool) True if all files are formatted else
            False.
    """
    filelist = []
    # walk all the folders from the current wotking directory
    for dir in include_folders:
        for root, dirs, files in os.walk(os.path.join(os.getcwd(), dir)):
            if os.path.relpath(root, os.getcwd()) in exclude_folders:
                files[:] = []
                dirs[:] = []

            # loop over files
            for file in files:
                # check for file extension
                if file.endswith(apply_extensions):
                    filelist.append(os.path.join(root, file))

    with multiprocessing.Pool(max_parallel_process) as p:
        try:
            return all(p.map(functools.partial(format_file, clangf_exe, dry_run=dry_run), filelist))
        except Exception as e:
            print("Error with formatting file", e)

    return False


def check_clang_format_version(clangf_exe):
    """
    Function checks the version of the clang-formatter
    available with the host system
    """
    try:
        _minimum_version_num = int(
            minimum_required_clang_format_version.replace('.', ''))
        if shutil.which(clangf_exe):
            ret = os.popen("{} --version".format(clangf_exe)).read()
            tokens = re.search("^(\w+-\w+) (\w+) ([0-9.]+).*", ret)
            if tokens.group(1) == "clang-format" and tokens.group(2) == "version":
                _current_version_num = int(tokens.group(3).replace(".", ""))
                if _current_version_num >= _minimum_version_num:
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False
    except:
        return False


# entry point
if __name__ == "__main__":
    # build a list of possible clang-format versions
    clang_executables = ['clang-format']
    for i in range(10, 99):
        clang_executables.append("clang-format-{}".format(i))

    # check if the appropriate one is installed
    clangf_exe = None
    for i in clang_executables:
        if check_clang_format_version(i):
            clangf_exe = i
            break
    # bail out in case no clang-format was found
    if clangf_exe is None:
        print("Please install clang-format version {}".format(minimum_required_clang_format_version))
        quit(1)

    dry_run = "check" in sys.argv

    # do the formatting
    ok = clang_format(clangf_exe, dry_run=dry_run)
    if ok:
        print(f"{'Formatting' if not dry_run else 'Checking'} done!")

    quit(0 if ok else 1)
