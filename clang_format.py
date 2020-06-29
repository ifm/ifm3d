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

"""
    Function to execute system command on console
        Args:
            command (string) : console command with parameters
        Return :
            standard output of command as a string
"""
def system_call(command):
    p = subprocess.run(command, stdout=subprocess.PIPE)
    return p.stdout.decode()

# Extension and folders to be excludes from formatting
apply_extensions = (".cxx",".cpp",".c", ".hxx", ".hh", ".cc", ".hpp", ".h")
exclude_folders = ("third-party",".github","build","cmake","docker","snap",".git","cxxopts", "contrib")

"""
    Function runs the clang formatter on the files
    with extension mention in  'apply_extensions' and
    not in the folders mentioned in 'exclude_folders'

    Return:
        (bool) True if all files are formatted else
               False.
"""
def clang_format():
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
                    os.system("clang-format -i -style=file " + os.path.join(root,file))
                    #print(os.path.join(root,file))
        return True
    except:
        print("Error with formatting file")
"""
    Function checks the version of the clang-formatter
    available with the host system
"""
def get_clang_format_version():
    ret = system_call (['clang-format', '--version'])
    tokens = re.search("^(\w+-\w+) (\w+) ([0-9.]+).*",ret)
    if tokens.group(1) == "clang-format" and tokens.group(2) == "version":
        return int(tokens.group(3).replace(".", ""))
    else:
        print("error getting clang-format version")

minimum_required_clang_format_version = 600

#entry point
if __name__ == "__main__":
    if get_clang_format_version() >= minimum_required_clang_format_version:
        if clang_format():
            print("Done formatting files")
    else :
        print("minimum required clang-format version is 6.0.0")
