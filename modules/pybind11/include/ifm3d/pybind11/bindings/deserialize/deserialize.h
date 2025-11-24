/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE
#define IFM3D_PYBIND_BINDING_DESERIALIZE

#include <ifm3d/pybind11/bindings/deserialize/global_deserializer.h>
#include <ifm3d/pybind11/bindings/deserialize/imu_info_v1.h>
#include <ifm3d/pybind11/bindings/deserialize/o3d_paramters.h>
#include <ifm3d/pybind11/bindings/deserialize/ods_extrinsic_calibration_correction_v1.h>
#include <ifm3d/pybind11/bindings/deserialize/ods_info_v1.h>
#include <ifm3d/pybind11/bindings/deserialize/ods_occupancy_grid_v1.h>
#include <ifm3d/pybind11/bindings/deserialize/ods_polar_occupancy_grid_v1.h>
#include <ifm3d/pybind11/bindings/deserialize/rgb_info_v1.h>
#include <ifm3d/pybind11/bindings/deserialize/tof_info_v3.h>
#include <ifm3d/pybind11/bindings/deserialize/tof_info_v4.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

inline void
bind_deserialize_struct(py::module_& m)
{
  bind_struct_tofinfov3(m);
  bind_struct_tofinfov4(m);
  bind_struct_rgbinfov1(m);
  bind_struct_odsinfov1(m);
  bind_struct_odsoccupancygridv1(m);
  bind_struct_o3d_paramters(m);
  bind_struct_odspolaroccupancygridv1(m);
  bind_struct_odsextrinsiccalibrationcorrectionv1(m);
  bind_deserializer(m);
  bind_struct_imuinfov1(m);
}

#endif // IFM3D_PYBIND_BINDING_DESERIALIZE