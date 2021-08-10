// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_MUTABLE_ARGS_H
#define IFM3D_TOOLS_MUTABLE_ARGS_H

#include <vector>

namespace ifm3d
{
  /**
   * Helper class to manage a (mutable) copy of argc/argv specifically
   * for use with the cxxopts library.
   */
  class MutableArgs
  {
  public:
    MutableArgs(int argc, const char** argv);
    ~MutableArgs();
    MutableArgs(const MutableArgs&) = delete;
    MutableArgs& operator=(const MutableArgs&) = delete;
    int argc;
    char** argv;

  private:
    std::vector<char*> in_argv_ptrs_;
  }; // end: class MutableArgs

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_MUTABLE_ARGS_H
