// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include <cstring>
#include <ifm3d/tools/mutable_args.h>

namespace ifm3d
{
  MutableArgs::MutableArgs(int argc, const char** argv)
  {
    this->argc = argc;

    this->argv = new char*[argc + 1];
    for (int i = 0; i < argc; ++i)
      {
        size_t length = strlen(argv[i]) + 1;
        this->argv[i] = new char[length];
        strncpy(this->argv[i], argv[i], length);
        in_argv_ptrs_.push_back(this->argv[i]);
      }
    this->argv[argc] = nullptr;
  }

  MutableArgs::~MutableArgs()
  {
    std::for_each(this->in_argv_ptrs_.begin(),
                  this->in_argv_ptrs_.end(),
                  [](char* v) {
                    if (v)
                      {
                        delete[] v;
                        v = nullptr;
                      }
                  });

    if (this->argv)
      {
        delete[] this->argv;
        this->argv = nullptr;
      }
  }

} // end: namespace ifm3d
