#
# Copyright 2021-present ifm electronic, gmbh
# SPDX-License-Identifier: Apache-2.0
#

import pytest
import time

import ifm3dpy

def test_waitforframe():
    cam = ifm3dpy.O3R()
    fg = ifm3dpy.FrameGrabber(cam, 50012)
    fg.start()
    wait_for_frame_timeout = 10000
    count = 0
    for i in range(10):
        res,frame = fg.wait_for_frame().wait_for(wait_for_frame_timeout)
        assert res == True , f"A timeout during wait_for_frame was detected. We did not capture data within {wait_for_frame_timeout} ms"
        count = count + 1
    assert count == 10



