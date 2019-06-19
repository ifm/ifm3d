#
# Copyright (C) 2019 ifm electronic, gmbh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import os.path
import re
import pytest
import ifm3dpy

def get_version_from_cmakelists():
    """
    Helper to parse the ifm3d package version out of the CMakeLists file

    This must be done out-of-band from building because we support direct
    installation through setuptools, pip, etc and cannot rely on the
    presence of auto-generated cmake variables.
    """

    # CMakeLists is either three levels up (if tests are run in place)
    # or four levels up (if being run through cmake in an out-of-source dir)
    # Check both locations.
    path_to_cmakelists = os.path.abspath('../../../CMakeLists.txt')
    if not os.path.isfile(path_to_cmakelists):
        path_to_cmakelists = os.path.abspath('../../../../CMakeLists.txt')
        if not os.path.isfile(path_to_cmakelists):
            raise RuntimeError('CMakeLists.txt not found, ' +
                               'unable to parse project version!')

    with open(path_to_cmakelists, "r") as f:
        lines = f.readlines()
    for line in lines:
        if "project(IFM3D VERSION" in line:
            for token in line.split(" "):
                if re.match(r'^[0-9]+\.[0-9]+\.[0-9]+$', token):
                    return token

def test_version():
    assert ifm3dpy.__version__ == get_version_from_cmakelists()

def test_name():
    assert ifm3dpy.__name__ == 'ifm3dpy'

def test_package():
    assert ifm3dpy.__package__ == 'ifm3dpy'
