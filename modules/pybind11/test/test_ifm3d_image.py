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
import pytest
import datetime
import time
import json
import numpy as np

import ifm3dpy

def test_factorydefaults():
    cam = ifm3dpy.Camera()
    cam.factory_reset()
    time.sleep(6)
    cam.device_type()

def test_references():
    cam = ifm3dpy.Camera()
    fg = ifm3dpy.FrameGrabber(cam)
    im = ifm3dpy.ImageBuffer()
    assert fg.wait_for_frame(im, 1000)

    amp1 = im.amplitude_image()
    amp2 = im.amplitude_image()

    if cam.is_O3X():
        assert amp1.dtype == np.float32
        assert amp2.dtype == np.float32
    else:
        assert amp1.dtype == np.uint16
        assert amp2.dtype == np.uint16

    assert (amp1 == amp2).all()
    amp2 += 1
    assert (amp1 == amp2).all()

def test_xyzimage():
    cam = ifm3dpy.Camera()
    fg = ifm3dpy.FrameGrabber(cam)
    im = ifm3dpy.ImageBuffer()
    assert fg.wait_for_frame(im, 1000)

    xyz = im.xyz_image()
    assert (xyz.dtype == np.int16) or (xyz.dtype == np.float32)
    assert (xyz.shape[2] == 3)

def test_computecartesian():
    cam = ifm3dpy.Camera()
    im = ifm3dpy.ImageBuffer()

    # 1. Stream in the unit vectors
    fg = ifm3dpy.FrameGrabber(cam, ifm3dpy.IMG_UVEC)
    assert fg.wait_for_frame(im, 1000)
    uvec = im.unit_vectors()

    # 2. Now we stream in both the radial distance image and the cartesian
    # data. The latter we simply use as ground truth
    fg.reset(cam, ifm3dpy.IMG_RDIS | ifm3dpy.IMG_CART)
    assert fg.wait_for_frame(im, 1000)
    rdis = im.distance_image()
    conf = im.confidence_image()
    xyz = im.xyz_image()

    if xyz.dtype == np.float32:
        # Convert to mm
        xyz *= 1000
        xyz = xyz.astype(np.int16)

    # We also need the translation vector from the extrinsics
    extrinsics = im.extrinsics()
    tx = extrinsics[0]
    ty = extrinsics[1]
    tz = extrinsics[2]

    # 3. Compute the cartesian data

    # unit vectors
    ex = uvec[:,:,0]
    ey = uvec[:,:,1]
    ez = uvec[:,:,2]


    rdis_f = rdis.copy().astype(np.float32)
    if rdis.dtype == np.float32:
        # Assume rdis was in meters, convert to mm
        rdis_f *= 1000

    # Compute
    x_ = ex * rdis_f + tx
    y_ = ey * rdis_f + ty
    z_ = ez * rdis_f + tz

    # Blank out bad pixels .. our zero pixels will
    # be exactly equal to tx, ty, tz and if any of those
    # exceed 1cm (our test tolerance) like on an O3D301,
    # we will get errors in the unit test.
    bad_mask = (np.bitwise_and(conf, 0x1) == 0x1)
    x_[bad_mask] = 0
    y_[bad_mask] = 0
    z_[bad_mask] = 0

    # 4. Cast (back) to int16 and transform to ifm3d coord frame
    x_i = x_.astype(np.int16)
    y_i = y_.astype(np.int16)
    z_i = z_.astype(np.int16)

    x_computed = z_i
    y_computed = -x_i
    z_computed = -y_i

    # 5. Compare for correctness
    assert (np.abs(x_computed - xyz[:,:,0]) <= 10).all()
    assert (np.abs(y_computed - xyz[:,:,1]) <= 10).all()
    assert (np.abs(z_computed - xyz[:,:,2]) <= 10).all()


def test_timestamp():
    json_str = (
        '{'
          '"o3d3xx":'
          '{'
            '"Device":'
            '{'
              '"ActiveApplication": "1"'
            '},'
            '"Apps":'
            '['
              '{'
                '"TriggerMode": "1",'
                '"Index": "1",'
                '"Imager":'
                '{'
                    '"ExposureTime": "5000",'
                    '"ExposureTimeList": "125;5000",'
                    '"ExposureTimeRatio": "40",'
                    '"Type":"under5m_moderate"'
                '}'
              '}'
            ']'
          '}'
        '}')
    j = json.loads(json_str)

    cam = ifm3dpy.Camera()
    cam.from_json(j)

    im = ifm3dpy.ImageBuffer()
    fg = ifm3dpy.FrameGrabber(cam, ifm3dpy.IMG_AMP | ifm3dpy.IMG_CART)

    # get two consecutive timestamps
    assert fg.wait_for_frame(im, 1000)
    tp1 = im.timestamp()
    assert fg.wait_for_frame(im, 1000)
    tp2 = im.timestamp()

    # the first time point needs to be smaller than the second one
    assert tp1 < tp2
    assert (tp2 - tp1) > datetime.timedelta(milliseconds = 20)

def test_memorymodel_ownership():
    cam = ifm3dpy.Camera()
    im = ifm3dpy.ImageBuffer()
    fg = ifm3dpy.FrameGrabber(cam)
    assert fg.wait_for_frame(im, 1000)

    # Memory is owned and managed in the C++ layer. When projected, numpy
    # should report that it does not 'own' the underlying buffer
    assert not im.xyz_image().flags.owndata
    assert not im.amplitude_image().flags.owndata

    # Pointer to underlying buffers should remain constant on consecutive
    # captures with the same ImageBuffer object
    addr_xyz1 = im.xyz_image().__array_interface__['data'][0]
    addr_amp1 = im.amplitude_image().__array_interface__['data'][0]
    assert fg.wait_for_frame(im, 1000)
    addr_xyz2 = im.xyz_image().__array_interface__['data'][0]
    addr_amp2 = im.amplitude_image().__array_interface__['data'][0]
    assert addr_xyz1 == addr_xyz2
    assert addr_amp1 == addr_amp2

    # Copies of numpy arrays should copy into the python memory space (aka
    # numpy should report that it DOES own memory of copies, and the addrs
    # should be different
    xyz = im.xyz_image()
    xyz_copy = xyz.copy()
    assert not xyz.flags.owndata
    assert xyz_copy.flags.owndata
    assert (xyz.__array_interface__['data'][0] !=
            xyz_copy.__array_interface__['data'][0])

def test_memorymodel_changing_flags():
    cam = ifm3dpy.Camera()
    im = ifm3dpy.ImageBuffer()

    # First grab a frame with only amplitude data
    fg = ifm3dpy.FrameGrabber(cam, ifm3dpy.IMG_AMP)
    assert fg.wait_for_frame(im, 1000)

    # XYZ should be empty, but have a valid address
    xyz = im.xyz_image()
    xyz_addr = xyz.__array_interface__['data'][0]
    assert all(i == 0 for i in xyz.shape)
    assert xyz_addr != 0
    amp = im.amplitude_image()
    amp_addr = amp.__array_interface__['data'][0]
    assert all(i != 0 for i in amp.shape)
    assert amp_addr != 0

    # Cache a copy of the data in amp
    amp_copy = amp.copy()

    # Now enable cartesian/xyz data and get another frame using the same
    # ImageBuffer object
    fg.reset(cam, ifm3dpy.IMG_AMP | ifm3dpy.IMG_CART)
    assert fg.wait_for_frame(im, 1000)

    # We expect XYZ to now be valid, (with a different pointer)
    xyz = im.xyz_image()
    assert all(i != 0 for i in xyz.shape)
    assert xyz_addr != xyz.__array_interface__['data'][0]
    xyz_addr = xyz.__array_interface__['data'][0]

    # Amplitude *data* will have changed, but not the underlying pointer!
    amp = im.amplitude_image()
    assert all(i != 0 for i in amp.shape)
    assert amp_addr == amp.__array_interface__['data'][0]
    assert (amp_copy != amp).any()
    amp_addr = amp.__array_interface__['data'][0]


