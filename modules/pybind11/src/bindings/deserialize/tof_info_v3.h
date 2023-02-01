/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_TOF_INFO_V3_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_TOF_INFO_V3_H

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ifm3d/deserialize/struct_tof_info_v3.hpp>

void
bind_struct_tofinfov3(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::calibration::ExtrinsicOpticToUser,
             ifm3d::calibration::ExtrinsicOpticToUser::Ptr> extrinsic_optic_to_user(
      m,
      "ExtrinsicOpticToUser",
      R"(
          Class for managing an instance Extrinsic parameters
       )");
  extrinsic_optic_to_user.def(
    py::init<>(),
    R"(
        Constructor
      )");
  extrinsic_optic_to_user.def_readonly(
    "trans_x",
    &ifm3d::calibration::ExtrinsicOpticToUser::trans_x,
    R"(
        Extrinsic Parameter transX
      )");
  extrinsic_optic_to_user.def_readonly(
    "trans_y",
    &ifm3d::calibration::ExtrinsicOpticToUser::trans_y,
    R"(
        Extrinsic Parameter transY
      )");
  extrinsic_optic_to_user.def_readonly(
    "trans_z",
    &ifm3d::calibration::ExtrinsicOpticToUser::trans_z,
    R"(
        Extrinsic Parameter transZ
      )");
  extrinsic_optic_to_user.def_readonly(
    "rot_x",
    &ifm3d::calibration::ExtrinsicOpticToUser::rot_x,
    R"(
        Extrinsic Parameter rotX
      )");
  extrinsic_optic_to_user.def_readonly(
    "rot_y",
    &ifm3d::calibration::ExtrinsicOpticToUser::rot_y,
    R"(
        Extrinsic Parameter rotY
      )");
  extrinsic_optic_to_user.def_readonly(
    "rot_z",
    &ifm3d::calibration::ExtrinsicOpticToUser::rot_z,
    R"(
        Extrinsic Parameter rotZ
      )");

  py::class_<ifm3d::calibration::Calibration, ifm3d::calibration::Calibration::Ptr> calibration(
    m,
    "Calibration",
    R"(
        Class for managing an instance calibration parameters
      )");
  calibration.def(
    py::init<>(),
    R"(
        Constructor
      )");
  calibration.def_readonly(
    "model_id",
    &ifm3d::calibration::Calibration::model_id,
    R"(
        Model Id for calibration parameters
      )");

  calibration.def_readonly(
    "parameters",
    &ifm3d::calibration::Calibration::model_parameters,
    R"(
        Parameters for calibration
      )");

  py::class_<ifm3d::TOFInfoV3, ifm3d::TOFInfoV3::Ptr> tof_info_v3(
    m,
    "TOFInfoV3",
    R"(
        Class for managing an instance of an struct TofInfoV3
      )");

  tof_info_v3.def(
    py::init<>(),
    R"(
        Constructor
      )");

  tof_info_v3.def_readonly(
    "version",
    &ifm3d::TOFInfoV3::version,
    R"(
         Version of the TOF_INFO data
      )");

  tof_info_v3.def_readonly(
    "amplitude_resolution",
    &ifm3d::TOFInfoV3::amplitude_resolution,
    R"(
         Resolution of the amplitude buffer
      )");
  tof_info_v3.def_readonly(
    "distance_resolution",
    &ifm3d::TOFInfoV3::distance_resolution,
    R"(
        Resolution of distance buffer per digit[m]
      )");
  tof_info_v3.def_readonly(
    "amp_normalization_factors",
    &ifm3d::TOFInfoV3::amp_normalization_factors,
    R"(
          Amplitude normalization factors for the individual exposure times
      )");
  tof_info_v3.def_readonly(
    "extrisic_optic_to_user",
    &ifm3d::TOFInfoV3::extrisic_optic_to_user,
    R"(
        Extrinsic optic parameter to user
      )");
  tof_info_v3.def_readonly(
    "intrinsic_calibration",
    &ifm3d::TOFInfoV3::intrinsic_calibration,
    R"(
        Intrinsic calibration parameters
      )");
  tof_info_v3.def_readonly(
    "inverse_intrinsic_calibration",
    &ifm3d::TOFInfoV3::inverse_intrinsic_calibration,
    R"(
        Inverse intrinsic calibration parameters
      )");
  tof_info_v3.def_readonly(
    "exposure_timestamps_ns",
    &ifm3d::TOFInfoV3::exposure_timestamps_ns,
    R"(
         The timestamp of the individual exposure time [nano seconds]
      )");
  tof_info_v3.def_readonly(
    "exposure_times_s",
    &ifm3d::TOFInfoV3::exposure_times_s,
    R"(
         Actual exposure times of a single phase image [seconds]
      )");
  tof_info_v3.def_readonly(
    "illu_temperature",
    &ifm3d::TOFInfoV3::illu_temperature,
    R"(
         Illumination temperature
      )");
  tof_info_v3.def_readonly(
    "mode",
    &ifm3d::TOFInfoV3::mode,
    R"(
        Mode of the head
      )");
  tof_info_v3.def_readonly(
    "imager",
    &ifm3d::TOFInfoV3::imager,
    R"(
        Imager type
      )");

  tof_info_v3.def_static(
    "deserialize",
    [](py::array_t<uint8_t, py::array::c_style | py::array::forcecast> in) -> ifm3d::TOFInfoV3 {
    ifm3d::TOFInfoV3 val;
    val.Read(reinterpret_cast<const uint8_t*>(in.data(0)),in.nbytes());
    return val;
  },
    R"(
        Deserialize TOF_INFO Buffer to ToFInfoV3 struct.
      )");
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_TOF_INFO_V3_H