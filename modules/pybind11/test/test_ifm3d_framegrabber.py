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
# distributed under the License is distribtued on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import pytest
import time
from multiprocessing.dummy import Pool as ThreadPool

import ifm3dpy

def test_factorydefaults():
    cam = ifm3dpy.Camera()
    cam.factory_reset()
    time.sleep(6)
    cam.device_type()

def test_waitforframe():
    cam = ifm3dpy.Camera()
    fg = ifm3dpy.FrameGrabber(cam)
    buff = ifm3dpy.ImageBuffer()

    count = 0
    for i in range(10):
        assert fg.wait_for_frame(buff, 1000)
        count = count + 1
    assert count == 10

def test_customschema():
    mask = ifm3dpy.IMG_AMP | ifm3dpy.IMG_RDIS | ifm3dpy.IMG_UVEC

    cam = ifm3dpy.Camera()
    fg = ifm3dpy.FrameGrabber(cam, mask)
    buff = ifm3dpy.ImageBuffer()
    assert fg.wait_for_frame(buff, 1000)


def test_intrinsicparamschema():
    mask = ifm3dpy.IMG_AMP | ifm3dpy.IMG_RDIS | ifm3dpy.INTR_CAL

    cam = ifm3dpy.Camera()
    if cam.is_O3X():
        with pytest.raises(RuntimeError):
            fg = ifm3dpy.FrameGrabber(cam, mask)
    elif (cam.is_O3D() and
             cam.check_minimum_firmware_version(
                ifm3dpy.O3D_INTRINSIC_PARAM_SUPPORT_MAJOR,
                ifm3dpy.O3D_INTRINSIC_PARAM_SUPPORT_MINOR,
                ifm3dpy.O3D_INTRINSIC_PARAM_SUPPORT_PATCH)):
        fg = ifm3dpy.FrameGrabber(cam, mask)
        buff = ifm3dpy.ImageBuffer()
        assert fg.wait_for_frame(buff, 1000)
    elif (cam.is_O3D()):
        with pytest.raises(RuntimeError):
            fg = ifm3dpy.FrameGrabber(cam, mask)

def test_inverseintrinsicparamschema():
    mask = (ifm3dpy.IMG_AMP | ifm3dpy.IMG_RDIS |
            ifm3dpy.INTR_CAL | ifm3dpy.INV_INTR_CAL)

    cam = ifm3dpy.Camera()
    if cam.is_O3X():
        with pytest.raises(RuntimeError):
            fg = ifm3dpy.FrameGrabber(cam, mask)
    elif (cam.is_O3D() and
             cam.check_minimum_firmware_version(
                ifm3dpy.O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR,
                ifm3dpy.O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR,
                ifm3dpy.O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH)):
        fg = ifm3dpy.FrameGrabber(cam, mask)
        buff = ifm3dpy.ImageBuffer()
        assert fg.wait_for_frame(buff, 1000)
    elif (cam.is_O3D()):
        with pytest.raises(RuntimeError):
            fg = ifm3dpy.FrameGrabber(cam, mask)

def test_framegrabberrecycling():
    cam = ifm3dpy.Camera()
    fg = ifm3dpy.FrameGrabber(cam)
    buff = ifm3dpy.ImageBuffer()

    for i in range(5):
        assert fg.wait_for_frame(buff, 1000)

    fg.reset(cam)

    for i in range(5):
        assert fg.wait_for_frame(buff, 1000)

def test_softwaretrigger():
    cam = ifm3dpy.Camera()
    idx = cam.active_application()
    config = cam.to_json()
    config['ifm3d']['Apps'][idx-1]['TriggerMode'] = \
        str(int(ifm3dpy.Camera.trigger_mode.SW))
    cam.from_json(config)

    fg = ifm3dpy.FrameGrabber(cam)
    buff = ifm3dpy.ImageBuffer()
    # waiting for an image should now timeout
    assert not fg.wait_for_frame(buff, 1000)

    # now, get image data by explicitly s/w triggering the device
    for i in range(10):
        fg.sw_trigger()
        assert fg.wait_for_frame(buff, 1000)

    # set the camera back into free-run mode
    config['ifm3d']['Apps'][idx-1]['TriggerMode'] = \
        str(int(ifm3dpy.Camera.trigger_mode.FREE_RUN))
    cam.from_json(config)

def test_swtriggermultipleclients():
    cam = ifm3dpy.Camera()

    # O3X cannot handle multiple client connections to PCIC
    # so this test does not apply
    if cam.is_O3X():
        return

    # mark the current active application as sw triggered
    idx = cam.active_application()
    config = cam.to_json()
    config['ifm3d']['Apps'][idx-1]['TriggerMode'] = \
        str(int(ifm3dpy.Camera.trigger_mode.SW))
    cam.from_json(config)

    # create two framegrabbers with same camera
    fg1 = ifm3dpy.FrameGrabber(cam)
    fg2 = ifm3dpy.FrameGrabber(cam)

    # launch two threads where each of the framegrabbers will
    # wait for a new frame
    def get_frame(fg):
        buff = ifm3dpy.ImageBuffer()
        if not fg.wait_for_frame(buff, 5000):
            buff = None
        return buff

    pool = ThreadPool(2)
    res = pool.map_async(get_frame, [fg1, fg2])

    # Let's S/W trigger from the first -- this could have been a third
    # framegrabber
    fg1.sw_trigger()
    pool.close()
    pool.join()

    # Did they both get a frame?
    frames = res.get()
    assert all(frames)

    # Check that the data are the same
    if all(frames):
        assert (frames[0].distance_image() ==
                frames[1].distance_image()).all()
        assert (frames[0].unit_vectors() ==
                frames[1].unit_vectors()).all()
        assert (frames[0].gray_image() ==
                frames[1].gray_image()).all()
        assert (frames[0].amplitude_image() ==
                frames[1].amplitude_image()).all()
        assert (frames[0].raw_amplitude_image() ==
                frames[1].raw_amplitude_image()).all()
        assert (frames[0].confidence_image() ==
                frames[1].confidence_image()).all()
        assert (frames[0].xyz_image() ==
               frames[1].xyz_image()).all()

    # set the camera back into free-run mode
    config['ifm3d']['Apps'][idx-1]['TriggerMode'] = \
        str(int(ifm3dpy.Camera.trigger_mode.FREE_RUN))
    cam.from_json(config)

def test_json_model():
    cam = ifm3dpy.Camera()
    mask = ifm3dpy.IMG_AMP | ifm3dpy.JSON_MODEL
    if cam.is_O3X():
        with pytest.raises(RuntimeError):
            fg = ifm3dpy.FrameGrabber(cam, mask)
    else:
        fg = ifm3dpy.FrameGrabber(cam, mask)
        buff = ifm3dpy.ImageBuffer()
        assert fg.wait_for_frame(buff, 1000)
        model = buff.json_model()
        assert model


