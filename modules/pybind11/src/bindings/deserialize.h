/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE
#define IFM3D_PYBIND_BINDING_DESERIALIZE

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <ifm3d/deserialize/struct_tof_info_v3.hpp>

void
bind_struct_tofinfov3(pybind11::module_& m)
{
  // clang-format off
  py::class_<ifm3d::TofInfoV3::ExtrinsicOpticToUser,
             ifm3d::TofInfoV3::ExtrinsicOpticToUser::Ptr>
    extrinsic_optic_to_user(
      m,
      "ExtrinsicOpticToUser",
      R"(Class for managing an instance Extrinsic parameters)");
  extrinsic_optic_to_user.def(py::init<>(), R"(Constructor)");
  extrinsic_optic_to_user.def_readonly(
    "transX",
    &ifm3d::TofInfoV3::ExtrinsicOpticToUser::transX,
    R"(Extrinsic Parameter transX)");
  extrinsic_optic_to_user.def_readonly(
    "transY",
    &ifm3d::TofInfoV3::ExtrinsicOpticToUser::transY,
    R"((Extrinsic Parameter transY)");
  extrinsic_optic_to_user.def_readonly(
    "transZ",
    &ifm3d::TofInfoV3::ExtrinsicOpticToUser::transZ,
    R"((Extrinsic Parameter transZ)");
  extrinsic_optic_to_user.def_readonly(
    "rotX",
    &ifm3d::TofInfoV3::ExtrinsicOpticToUser::rotX,
    R"((Extrinsic Parameter rotX)");
  extrinsic_optic_to_user.def_readonly(
    "rotY",
    &ifm3d::TofInfoV3::ExtrinsicOpticToUser::rotY,
    R"(Extrinsic Parameter rotY)");
  extrinsic_optic_to_user.def_readonly(
    "rotZ",
    &ifm3d::TofInfoV3::ExtrinsicOpticToUser::rotZ,
    R"(Extrinsic Parameter rotZ)");

  py::class_<ifm3d::TofInfoV3::Calibration, ifm3d::TofInfoV3::Calibration::Ptr>
    Calibration(m,
                "Calibration",
                R"(Class for managing an instance calibration parameters)");
  Calibration.def(py::init<>(), R"(Constructor)");
  Calibration.def_readonly("model_id",
                           &ifm3d::TofInfoV3::Calibration::modelID,
                           R"(Model Id for calibration parameters)");

  Calibration.def_readonly("parameters",
                           &ifm3d::TofInfoV3::Calibration::modelParameters,
                           R"(Parameters for calibration)");

  py::class_<ifm3d::TofInfoV3, ifm3d::TofInfoV3::Ptr> tof_info_v3(
    m,
    "ToFInfoV3",
    R"(Class for managing an instance of an struct TofInfoV3)");

  tof_info_v3.def(py::init<>(), R"(Constructor)");

  tof_info_v3.def_readonly("version",
                           &ifm3d::TofInfoV3::version,
                           R"(Version of the TOF_INFO)");

  tof_info_v3.def_readonly("amplitude_resolution",
                           &ifm3d::TofInfoV3::amplitude_resolution,
                           R"(Amplitude Resolution)");
  tof_info_v3.def_readonly("distance_resolution",
                           &ifm3d::TofInfoV3::distance_resolution,
                           R"(Distance Resolution)");
  tof_info_v3.def_readonly("amp_normalization_factors",
                           &ifm3d::TofInfoV3::amp_normalization_factors,
                           R"(Amplitude Resolution factors)");
  tof_info_v3.def_readonly("extrisic_optic_to_user",
                           &ifm3d::TofInfoV3::extrisic_optic_to_user,
                           R"(Extrinsic optic parameter to user)");
  tof_info_v3.def_readonly("intrinsic_calibration",
                           &ifm3d::TofInfoV3::intrinsic_calibration,
                           R"(intrinsic calibration paramter)");
  tof_info_v3.def_readonly("inverse_intrinsic_calibration",
                           &ifm3d::TofInfoV3::inverse_intrinsic_calibration,
                           R"(inverse intrinsic calibration paramter)");
  tof_info_v3.def_readonly("exposure_timestamps_ns",
                           &ifm3d::TofInfoV3::exposure_timestamps_ns,
                           R"(exposure timestamp in nano seconds)");
  tof_info_v3.def_readonly("exposure_times_s",
                           &ifm3d::TofInfoV3::exposure_times_s,
                           R"(exposure times in seconds)");
  tof_info_v3.def_readonly("illu_temperature",
                           &ifm3d::TofInfoV3::illu_temperature,
                           R"(illumination temperature)");
  tof_info_v3.def_readonly("mode",
                           &ifm3d::TofInfoV3::mode,
                           R"(mode of imager)");
  tof_info_v3.def_readonly("imager",
                           &ifm3d::TofInfoV3::imager,
                           R"(imager)");

  tof_info_v3.def_static("Deserialize", [](py::array in) -> ifm3d::TofInfoV3 {
    ifm3d::TofInfoV3 val;
    val.Read(reinterpret_cast<const uint8_t*>(in.data<uint8_t>(0)));
    return val;
  });
}
  
void
bind_deserialize_struct(pybind11::module_& m)
{
  bind_struct_tofinfov3(m);
}
// clang-format on
#endif // IFM3D_PYBIND_BINDING_DESERIALIZE