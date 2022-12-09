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



