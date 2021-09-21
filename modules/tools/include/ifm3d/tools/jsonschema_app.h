// -*- c++ -*-
/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_TOOLS_JSONSCHEMA_APP_H
#define IFM3D_TOOLS_JSONSCHEMA_APP_H

#include <string>
#include <ifm3d/tools/cmdline_app.h>

namespace ifm3d
{
  /**
   * Concrete implementation of the `schema` subcommand to the `ifm3d`
   * command-line utility.
   */
  class JsonSchemaApp : public ifm3d::CmdLineApp
  {
  public:
    JsonSchemaApp(int argc,
                  const char** argv,
                  const std::string& name = "jsonschema");
    int Run() override;
    bool CheckCompatibility() override;
  }; // end: class JsonSchemaApp

} // end: namespace ifm3d

#endif // IFM3D_TOOLS_JSONSCHEMA_APP_H
