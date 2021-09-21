/*
 * Copyright 2021 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/jsonschema_app.h>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/camera/camera_o3r.h>

ifm3d::JsonSchemaApp::JsonSchemaApp(int argc,
                                    const char** argv,
                                    const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::JsonSchemaApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  json schema =
    std::static_pointer_cast<ifm3d::O3RCamera>(this->cam_)->Schema();
  std::cout << schema.dump(2) << std::endl;

  return 0;
}

bool
ifm3d::JsonSchemaApp::CheckCompatibility()
{
  return this->cam_->AmI(CameraBase::device_family::O3R);
}