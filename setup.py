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
import os
import re
import sys
import platform
import subprocess

from distutils.version import LooseVersion
from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext

#
# This setup script was borrowed and modified from the pybind11 sample
# project found here: https://github.com/pybind/cmake_example
#


def get_version_from_git():
    """
    Helper to get the ifm3d package version from git
    """
    try:
        subprocess.check_call(['git', '--version'])
    except OSError:
        return "0.0.0"

    version = subprocess.check_output(
        ["git", "describe", "--abbrev=0", "--tags"]).decode("utf-8").strip()

    version_ahead = int(subprocess.check_output(
        ["git", "rev-list", version + "..HEAD", "--count"]).decode("utf-8").strip())

    if version_ahead > 0:
        # PyPI does not allow uploading versions with metadata so we can't include the commit hash...
        version = "{}-{}".format(version, version_ahead)

    version = version.lstrip("v")

    return version


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(
                re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(
            self.get_ext_fullpath(ext.name)))

        # Build with cmake -- build only camera and framegrabber. Also build
        # them as static libs so the resulting python module is isolated.
        cmake_args = ['-DBUILD_MODULE_IMAGE=OFF',
                      '-DBUILD_MODULE_SWUPDATER=ON',
                      '-DBUILD_MODULE_TOOLS=ON',
                      '-DBUILD_MODULE_PYBIND11=ON',
                      '-DBUILD_TESTS=OFF',
                      '-DBUILD_SHARED_LIBS=OFF',
                      '-DCMAKE_USE_OPENSSL=OFF',
                      '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_ARCHIVE_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable,
                      '-DCREATE_PYTHON_STUBS=OFF']

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            cmake_args += [
                '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]

            if 'IFM3D_BUILD_DIR' in os.environ:
                cmake_args += ['-DCMAKE_PREFIX_PATH=' +
                               os.environ['IFM3D_BUILD_DIR'].replace('"', '') + '\\install']

            # If a generator was specified, use it. Otherwise use the machine's
            # architecture and the default generator.
            if 'IFM3D_CMAKE_GENERATOR' in os.environ:
                cmake_args += ['-G',
                               os.environ['IFM3D_CMAKE_GENERATOR'].replace('"', '')]
            elif sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']

            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            cmake_args += ['-DCMAKE_POSITION_INDEPENDENT_CODE=ON']
            build_args += ['--']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] +
                              cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.', f'-j{os.cpu_count()}'] +
                              build_args, cwd=self.build_temp)

# Read the contents of README file
def read_description(fname):
    return open(os.path.join(os.path.dirname(__file__), fname), encoding='utf-8').read()

setup(
    name='ifm3dpy',
    version=get_version_from_git(),
    author='ifm Robotics Perception',
    author_email='support.robotics@ifm.com',
    description='Library for working with ifm pmd-based 3D ToF Cameras',
    url='https://github.com/ifm/ifm3d',
    license='Apache 2.0',
    long_description=read_description("README.md"),
    long_description_content_type='text/markdown',
    ext_modules=[CMakeExtension('IFM3D_PYBIND11')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
    packages=find_packages(),
    entry_points={'console_scripts': ['ifm3dpy = ifm3dpy:_run_cmdtool']},
    install_requires=['numpy'],
    project_urls={
        'Documentation': 'https://ifm3d.com/',
        'Issue Tracker': 'https://github.com/ifm/ifm3d/issues',
        'Source Code': 'https://github.com/ifm/ifm3d'
    }
)
