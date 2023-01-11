/*
 * Copyright 2023-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_PYBIND_BINDING_DESERIALIZE
#define IFM3D_PYBIND_BINDING_DESERIALIZE

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include "tof_info_v3.h"
void
bind_deserialize_struct(pybind11::module_& m)
{
  bind_struct_tofinfov3(m);
}

#endif // IFM3D_PYBIND_BINDING_DESERIALIZE