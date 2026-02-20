/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_IMU_INFO_V1_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_IMU_INFO_V1_H

#include <ifm3d/deserialize/struct_imu_info_v1.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

inline void
bind_struct_imuinfov1(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::calibration::IMUSample,
            ifm3d::calibration::IMUSample::Ptr> imu_sample(m,
    "IMUSample",
    R"(
        Class for managing an instance of an struct IMUSample
    )");

  imu_sample.def(py::init<>(),
    R"(
        Constructor
    )");
  imu_sample.def_readonly("hw_timestamp",
    &ifm3d::calibration::IMUSample::hw_timestamp,
    R"(
        Hardware timestamp of the IMU sample
    )");
  imu_sample.def_readonly("timestamp",
    &ifm3d::calibration::IMUSample::timestamp,
    R"(
        Timestamp of the IMU sample
    )");
  imu_sample.def_readonly("temperature",
    &ifm3d::calibration::IMUSample::temperature,
    R"(
        Temperature at the time of the IMU sample
    )");
  imu_sample.def_readonly("acc_x",
    &ifm3d::calibration::IMUSample::acc_x,
    R"(
        Acceleration in x direction in [m/s^2]
    )");
  imu_sample.def_readonly("acc_y",
    &ifm3d::calibration::IMUSample::acc_y,
    R"(
        Acceleration in y direction in [m/s^2]
    )");
  imu_sample.def_readonly("acc_z",
    &ifm3d::calibration::IMUSample::acc_z,
    R"(
        Acceleration in z direction in [m/s^2]
    )");
  imu_sample.def_readonly("gyro_x",
    &ifm3d::calibration::IMUSample::gyro_x,
    R"(
        Gyroscope x axis in [rad/s]
    )");
  imu_sample.def_readonly("gyro_y",
    &ifm3d::calibration::IMUSample::gyro_y,
    R"(
        Gyroscope y axis in [rad/s]
    )");
  imu_sample.def_readonly("gyro_z",
    &ifm3d::calibration::IMUSample::gyro_z,
    R"(
        Gyroscope z axis in [rad/s]
    )");

  py::class_<ifm3d::IMUInfoV1, ifm3d::IMUInfoV1::Ptr> imu_info_v1(m,
    "IMUInfoV1",
    R"(
        Class for managing an instance of an struct IMUInfoV1
    )");

  imu_info_v1.def(py::init<>(),
    R"(
        Constructor
    )");

  imu_info_v1.def_readonly("imu_version",
    &ifm3d::IMUInfoV1::imu_version,
    R"(
        Version of the IMU_INFO data
    )");

  imu_info_v1.def_readonly("imu_samples",
    &ifm3d::IMUInfoV1::imu_samples,
    R"(
        Array of IMU samples containing accelerometer and gyroscope data
    )");

  imu_info_v1.def_readonly("num_samples",
    &ifm3d::IMUInfoV1::num_samples,
    R"(
        Number of valid samples in imu_samples array
    )");

  imu_info_v1.def_readonly("extrinsic_imu_to_user",
    &ifm3d::IMUInfoV1::extrinsic_imu_to_user,
    R"(
        Extrinsic calibration for converting between IMU and User coordinates (combines both, i.e., IMUToVPU and VPUToUser, extrinsic calibrations)
    )");

  imu_info_v1.def_readonly("extrinsic_imu_to_vpu",
    &ifm3d::IMUInfoV1::extrinsic_imu_to_vpu,
    R"(
        Extrinsic calibration for converting between IMU and User coordinates (only the IMUToVPU calibration)
    )");

  imu_info_v1.def_readonly("imu_fifo_rcv_timestamp",
    &ifm3d::IMUInfoV1::imu_fifo_rcv_timestamp,
    R"(
        The receive timestamp of the IMU FIFO buffer. This is taken as soon as possible after the data has been arrived and might be used for manual synchronization of the IMU's hardware timestamps
    )");

  imu_info_v1.def_static(
    "deserialize",
    [](const py::array_t<uint8_t, py::array::c_style | py::array::forcecast>&
         in) -> ifm3d::IMUInfoV1 {
      ifm3d::IMUInfoV1 val{};
      val.Read(reinterpret_cast<const uint8_t*>(in.data(0)), in.size());
      return val;
    },
    R"(
        Deserialize a IMUInfoV1 structure from a byte array
    )");
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_IMU_INFO_V1_H