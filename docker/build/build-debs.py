#!/usr/bin/env python3
# -*- python -*-

#
# Copyright (C)  2019 ifm electronic, gmbh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distribted on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import argparse
import os
import subprocess
import sys
import time

DOCKER_FILE = 'Dockerfile'
DOCKER_HOME_DIR = '/home/ifm'

def get_args():
    parser = argparse.ArgumentParser(
      description='Build ifm3d debs via Docker')

    parser.add_argument('--docker_file', nargs='+', required=False,
                            default=[],
                            help="""DockerFile list to process. If not
                            specified, all `DockerFile' specifications
                            encountered in all subirectories (with respect to
                            your current working directory), recursively, will
                            be processed.""")
    parser.add_argument('--image', required=False, type=str,
                            default='ifm3d', help="""Docker image name.""")
    parser.add_argument('--vtag', required=False, type=str,
                            help="""ifm3d git version tag to build. If omitted,
                            `master' will be checked out and built.""")
    parser.add_argument('--run_tests', action='store_true',
                            help="""Run unit tests during build. Only specify
                            this flag if testing against live hardware (e.g.,
                            O3D or O3X).""")

    args = parser.parse_args(sys.argv[1:])
    return args

def sh(cmd):
    def sh_(cmd):
        popen = subprocess.Popen(cmd,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT,
                                 universal_newlines=True)
        for line in iter(popen.stdout.readline, ""):
            yield line
        popen.stdout.close()

        while popen.poll() is None:
            time.sleep(.1)
        if popen.returncode != 0:
            raise RuntimeError("%s: %s" % (' '.join(cmd), popen.returncode))

    for line in sh_(cmd):
        print(line, end="")

def run(cmd):
    print(' '.join(cmd))
    retval = subprocess.run(cmd,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            universal_newlines=True)
    print(str(retval))
    if retval.returncode != 0:
        raise RuntimeError()

    return retval

def docker_build(docker_file, docker_image, vtag, run_tests):
    old_cwd = os.getcwd()
    print("cwd is %s" % old_cwd)

    print("Attempting to build %s..." % docker_file)
    d_file = os.path.basename(docker_file)
    os.chdir(os.path.dirname(docker_file))
    new_cwd = os.getcwd()
    print("cwd is now %s" % new_cwd)

    # build the docker image
    cmd = ["docker", "build", "--no-cache", "-t", docker_image, "."]
    if vtag is not None:
        cmd.insert(2, "--build-arg")
        cmd.insert(3, "vtag=%s" % vtag)
    if run_tests:
        cmd.insert(2, "--build-arg")
        cmd.insert(3, "run_tests=1")
    print(' '.join(cmd))
    sh(cmd)

    # start a container using the new image
    retval = run(["docker", "run", "-dit", docker_image, "/bin/bash"])
    cid = retval.stdout.rstrip()

    # since `docker cp` cannot deal with globbing, we move the
    # debs to a specific folder, then, copy the folder in.
    if "ros" in docker_file:
        run(["docker", "exec", cid, "/bin/bash", "-c",
              "mkdir -p debs/; \
               mv src/ifm3d/build/*.deb debs/"])
    else:
        run(["docker", "exec", cid, "/bin/bash", "-c",
              "mkdir -p debs/; \
               mv src/ifm3d/build/*.deb debs/; \
               mv src/ifm3d-pcl-viewer/build/*.deb debs/"])

    run(["docker", "cp", "%s:%s/debs" % (cid, DOCKER_HOME_DIR), "."])

    # clean up
    run(["docker", "rm", "-f", cid])
    run(["docker", "rmi", "-f", docker_image])

    print("Changing back to %s" % old_cwd)
    os.chdir(old_cwd)

def main():
    args = get_args()

    docker_image = args.image
    vtag = args.vtag
    run_tests = args.run_tests
    docker_file = args.docker_file
    if len(docker_file) == 0:
        docker_file = []
        for dirpath, dirnames, filenames in os.walk("."):
            if DOCKER_FILE in filenames:
                docker_file.append(os.path.join(dirpath, DOCKER_FILE))

    for d in docker_file:
        docker_build(d, docker_image, vtag, run_tests)

    return 0

if __name__ == '__main__':
    sys.exit(main())
