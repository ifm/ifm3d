/*
 * Copyright 2025-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_EXTRINSIC_CALIBRATION_CORRECTION_V1_H
#define IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_EXTRINSIC_CALIBRATION_CORRECTION_V1_H

#include <ifm3d/deserialize/struct_o3r_ods_extrinsic_calibration_correction_v1.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

inline void
bind_struct_odsextrinsiccalibrationcorrectionv1(pybind11::module_& m)
{
  py::class_<ifm3d::ODSExtrinsicCalibrationCorrectionV1,
             ifm3d::ODSExtrinsicCalibrationCorrectionV1::Ptr>
    ods_extrinsic_calibration_correction_v1(
      m,
      "ODSExtrinsicCalibrationCorrectionV1",
      R"(
        Class for managing an instance of an struct ODSExtrinsicCalibrationCorrectionV1
      )");

  ods_extrinsic_calibration_correction_v1.def(py::init<>(),
                                              R"(
          Constructor
        )");

  ods_extrinsic_calibration_correction_v1.def_readonly(
    "version",
    &ifm3d::ODSExtrinsicCalibrationCorrectionV1::version,
    R"(
          version of the grid
        )");

  ods_extrinsic_calibration_correction_v1.def_readonly(
    "completion_rate",
    &ifm3d::ODSExtrinsicCalibrationCorrectionV1::completion_rate,
    R"(
          The completion rate ranging from 0.0 to 1.0, where 1.0 indicates that
          sufficient data was collected to estimate all three delta values.
        )");

  ods_extrinsic_calibration_correction_v1.def_readonly(
    "rot_delta_value",
    &ifm3d::ODSExtrinsicCalibrationCorrectionV1::rot_delta_value,
    R"(
          The estimated rot delta value [rad] to correct the extrinsic calibration.
          Array of [X, Y, Z].
        )");

  ods_extrinsic_calibration_correction_v1.def_readonly(
    "rot_delta_valid",
    &ifm3d::ODSExtrinsicCalibrationCorrectionV1::rot_delta_valid,
    R"(
          A flag indicating a valid estimation of rotation delta value (0: invalid, 1: valid).
          Array of [X, Y, Z].
        )");

  ods_extrinsic_calibration_correction_v1.def_readonly(
    "rot_head_to_user",
    &ifm3d::ODSExtrinsicCalibrationCorrectionV1::rot_head_to_user,
    R"(
          The rotation value [rad] of the (corrected) extrinsic calibration (extrinsicHeadToUser).
          Array of [X, Y, Z].
        )");

  ods_extrinsic_calibration_correction_v1.def_static(
    "deserialize",
    [](const py::array_t<uint8_t, py::array::c_style | py::array::forcecast>&
         in) -> ifm3d::ODSExtrinsicCalibrationCorrectionV1 {
      ifm3d::ODSExtrinsicCalibrationCorrectionV1 val{};
      val.Read(reinterpret_cast<const uint8_t*>(in.data(0)), in.nbytes());
      return val;
    },
    R"(
        Deserialize ODSExtrinsicCalibrationCorrectionV1 Buffer
      )");
}

#endif // IFM3D_PYBIND_BINDING_DESERIALIZE_ODS_EXTRINSIC_CALIBRATION_CORRECTION_V1_H