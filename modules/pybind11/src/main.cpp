/*
 * Copyright 2019 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/pybind11/bindings/ifm3d.h>
#include <ifm3d/pybind11/util.hpp> // NOLINT(misc-include-cleaner), included so clangd can infer the correct compilation settings
#include <pybind11/pybind11.h> // NOLINT(misc-include-cleaner), included for PYBIND11_MODULE

PYBIND11_MODULE(ifm3dpy, m) // NOLINT(misc-include-cleaner)
{
  // Bind the ifm3d module
  bind_ifm3d(m);
}
