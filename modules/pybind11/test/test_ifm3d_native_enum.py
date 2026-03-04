#
# Copyright 2026-present ifm electronic, gmbh
# SPDX-License-Identifier: Apache-2.0
#

from enum import IntEnum, IntFlag
import os
import pytest
from ifm3dpy.device import Device, O3R, Parameter, SetTemporaryApplicationParameter
from ifm3dpy.framegrabber import FrameGrabber, buffer_id
from ifm3dpy.logging import LogLevel

IP_ADDRESS = "192.168.0.69"

@pytest.mark.parametrize(
    "enum_type",
    [
        buffer_id,
        Device.boot_mode,
        Device.operating_mode,
        Device.trigger_mode,
        Device.spatial_filter,
        Device.temporal_filter,
        Device.mfilt_mask_size,
        Device.device_family,
        Parameter,
        LogLevel,
    ],
)
def test_native_intenum_types(enum_type):
    assert issubclass(enum_type, IntEnum)


def test_native_intflag_type():
    assert issubclass(Device.import_flags, IntFlag)


@pytest.mark.parametrize(
    "enum_value",
    [
        buffer_id.RADIAL_DISTANCE_IMAGE,
        Device.boot_mode.PRODUCTIVE,
        Device.operating_mode.RUN,
        Device.trigger_mode.FREE_RUN,
        Device.spatial_filter.MEDIAN,
        Device.temporal_filter.MEAN,
        Device.mfilt_mask_size._3x3,
        Device.device_family.O3R,
        Parameter.ODS_MOTION_DATA,
        LogLevel.Warning,
        Device.import_flags.APPS,
    ],
)
def test_native_enum_members_are_int_compatible(enum_value):
    assert isinstance(enum_value, int)
    assert int(enum_value) >= 0


def test_parameter_enum_value_mapping():
    assert int(Parameter.ODS_OVERHANGING_LOAD) == 2003
    assert int(Parameter.ODS_ZONE_SET) == 2101
    assert int(Parameter.ODS_MAXIMUM_HEIGHT) == 2102
    assert int(Parameter.ODS_MOTION_DATA) == 2103
    assert int(Parameter.PDS_GET_PALLET) == 2200
    assert int(Parameter.PDS_GET_ITEM) == 2201
    assert int(Parameter.PDS_GET_RACK) == 2202
    assert int(Parameter.PDS_VOL_CHECK) == 2203


def test_native_enums_on_live_o3r():
    dev = O3R(IP_ADDRESS)

    frame = None
    for port in (50012, 50013):
        fg = FrameGrabber(dev, port)
        fg.start()
        try:
            ok, candidate = fg.wait_for_frame().wait_for(2500)
            frame = candidate if ok else None
        except Exception:
            frame = None
        fg.stop()

        if frame is not None:
            break

    assert frame is not None, "No frame received from O3R on tested 3D ports"

    buffer = frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
    assert buffer is not None, "RADIAL_DISTANCE_IMAGE buffer not available"
