#!/usr/bin/python3

# SPDX-License-Identifier: Apache-2.0
# Copyright (C) ifm electronic gmbh
#
# THE PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.

# Provides functions to format C/C++ files using clang-format
import sys
assert sys.version_info > (3, 5), "python 3.5 or more required"
import os
import subprocess
import re
import shutil

# The bare Minimum clang-format version
minimum_required_clang_format_version = "10.0.0"


# Extension and folders to be excludes from formatting
apply_extensions = (".cxx",".cpp",".c", ".hxx", ".hh", ".cc", ".hpp", ".h")
exclude_folders = ("third-party",".github","build","cmake","docker","snap",".git","cxxopts", "contrib", "build_2019", "build_2017", "build_debug" , "build_release" , "examples")

"""
    Function runs the clang formatter on the files
    with extension mention in  'apply_extensions' and
    not in the folders mentioned in 'exclude_folders'

    Return:
        (bool) True if all files are formatted else
               False.
"""
def clang_format(clangf_exe):
    try:
        # walk all the folders from the current wotking directory
        for root, dirs, files in os.walk(os.getcwd()):
            # check if the folder is in excluded list
            if os.path.split(root)[1] in exclude_folders :
                files[:]=[]
                dirs[:] = []
            #loop over files
            for file in files:
                #check for file extension
                if file.endswith(apply_extensions):
                    os.system("{} -i -style=file {}".format(clangf_exe,os.path.join(root,file)))
        return True
    except:
        print("Error with formatting file")

"""
    Function checks the version of the clang-formatter
    available with the host system
"""
def check_clang_format_version(clangf_exe):
    try:
        _minimum_version_num = int(minimum_required_clang_format_version.replace('.',''))
        if shutil.which(clangf_exe):
            ret = os.popen("{} --version".format(clangf_exe)).read()
            tokens = re.search("^(\w+-\w+) (\w+) ([0-9.]+).*",ret)
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


#entry point
if __name__ == "__main__":
    # build a list of possible clang-format versions
    clang_executables = ['clang-format']
    for i in range(10,99):
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

    # do the formatting
    if clang_format(clangf_exe):
        print("Formatting Done!")
