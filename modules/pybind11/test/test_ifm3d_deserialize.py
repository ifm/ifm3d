#
# Copyright 2025-present ifm electronic, gmbh
# SPDX-License-Identifier: Apache-2.0
#

import pytest
from ifm3dpy.device import O3R
from ifm3dpy.framegrabber import FrameGrabber, buffer_id
from ifm3dpy.deserialize import deserialize, TOFInfoV3, TOFInfoV4, RGBInfoV1, \
    ODSOccupancyGridV1, ODSInfoV1, IMUInfoV1

DEVICE_IP = "192.168.0.69"
FRAME_TIMEOUT_MS = 5000

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


def test_deserialize_imu_info_v1():
    dev = O3R(DEVICE_IP)
    pcic_port = dev.port("port6").pcic_port
    fg = FrameGrabber(cam=dev, pcic_port=pcic_port)

    fg.start([buffer_id.O3R_RESULT_IMU])

    ok, frame = fg.wait_for_frame().wait_for(FRAME_TIMEOUT_MS)
    assert ok, f"Timed out waiting for IMU frame from {DEVICE_IP}"
    assert frame is not None, "No frame received"

    buffer = frame.get_buffer(buffer_id.O3R_RESULT_IMU)
    assert buffer is not None, "O3R_RESULT_IMU buffer not available"

    result = deserialize(buffer)
    assert result is not None, "Deserialization returned None"
    assert isinstance(result, IMUInfoV1), f"Unexpected type: {type(result)}"

    assert isinstance(result.imu_version, int)
    assert isinstance(result.num_samples, int)
    assert 0 <= result.num_samples <= 128
    assert isinstance(result.imu_samples, list)
    assert len(result.imu_samples) == 128
    assert isinstance(result.imu_fifo_rcv_timestamp, int)

    assert hasattr(result, "extrinsic_imu_to_user")
    assert hasattr(result, "extrinsic_imu_to_vpu")

    assert isinstance(result.extrinsic_imu_to_user.trans_x, float)
    assert isinstance(result.extrinsic_imu_to_user.trans_y, float)
    assert isinstance(result.extrinsic_imu_to_user.trans_z, float)
    assert isinstance(result.extrinsic_imu_to_user.rot_x, float)
    assert isinstance(result.extrinsic_imu_to_user.rot_y, float)
    assert isinstance(result.extrinsic_imu_to_user.rot_z, float)

    assert isinstance(result.extrinsic_imu_to_vpu.trans_x, float)
    assert isinstance(result.extrinsic_imu_to_vpu.trans_y, float)
    assert isinstance(result.extrinsic_imu_to_vpu.trans_z, float)
    assert isinstance(result.extrinsic_imu_to_vpu.rot_x, float)
    assert isinstance(result.extrinsic_imu_to_vpu.rot_y, float)
    assert isinstance(result.extrinsic_imu_to_vpu.rot_z, float)

    sample = result.imu_samples[0]
    assert isinstance(sample.hw_timestamp, int)
    assert isinstance(sample.timestamp, int)
    assert isinstance(sample.temperature, float)
    assert isinstance(sample.acc_x, float)
    assert isinstance(sample.acc_y, float)
    assert isinstance(sample.acc_z, float)
    assert isinstance(sample.gyro_x, float)
    assert isinstance(sample.gyro_y, float)
    assert isinstance(sample.gyro_z, float)
