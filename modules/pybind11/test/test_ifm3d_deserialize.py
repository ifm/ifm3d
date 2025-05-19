#
# Copyright 2025-present ifm electronic, gmbh
# SPDX-License-Identifier: Apache-2.0
#

import pytest
from ifm3dpy.device import O3R
from ifm3dpy.framegrabber import FrameGrabber, buffer_id
from ifm3dpy.deserialize import deserialize, TOFInfoV3, TOFInfoV4, RGBInfoV1, \
    ODSOccupancyGridV1, ODSInfoV1

def test_deserialize_tof_info():
    dev = O3R()
    fg = FrameGrabber(dev, 50012)

    fg.start()

    frame = fg.wait_for_frame().wait()
    assert frame is not None, "No frame received"

    buffer = frame.get_buffer(buffer_id.TOF_INFO)
    assert buffer is not None, "TOF_INFO buffer not available"

    result = deserialize(buffer)
    assert result is not None, "Deserialization returned None"

    assert isinstance(result, (TOFInfoV3, TOFInfoV4)), \
    f"Unexpected type: {type(result)}"

def test_deserialize_rgb_info_v1():
    dev = O3R()
    fg = FrameGrabber(dev, 50010)

    fg.start()

    frame = fg.wait_for_frame().wait()

    assert frame is not None, "No frame received"

    buffer = frame.get_buffer(buffer_id.RGB_INFO)
    assert buffer is not None, "TOF_INFO buffer not available"

    result = deserialize(buffer)
    assert result is not None, "Deserialization returned None"

    assert isinstance(result, RGBInfoV1), f"Unexpected type: {type(result)}"

def test_deserialize_ods_info_v1():
    dev = O3R()
    fg = FrameGrabber(dev, 51010)

    fg.start()

    frame = fg.wait_for_frame().wait()
    assert frame is not None, "No frame received"

    buffer = frame.get_buffer(buffer_id.O3R_ODS_INFO)
    assert buffer is not None, "TOF_INFO buffer not available"

    result = deserialize(buffer)
    assert result is not None, "Deserialization returned None"

    assert isinstance(result, ODSInfoV1), f"Unexpected type: {type(result)}"

def test_deserialize_ods_occupancy_grid_v1():
    dev = O3R()
    fg = FrameGrabber(dev, 51010)

    fg.start()

    frame = fg.wait_for_frame().wait()
    assert frame is not None, "No frame received"

    buffer = frame.get_buffer(buffer_id.O3R_ODS_OCCUPANCY_GRID)
    assert buffer is not None, "TOF_INFO buffer not available"

    result = deserialize(buffer)
    assert result is not None, "Deserialization returned None"

    assert isinstance(result, ODSOccupancyGridV1), \
    f"Unexpected type: {type(result)}"
