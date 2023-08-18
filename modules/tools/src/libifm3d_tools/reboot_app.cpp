/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/reboot_app.h>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/device/device.h>
#include <ifm3d/swupdater.h>

ifm3d::RebootApp::RebootApp(int argc,
                            const char** argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name, false)
{
  // clang-format off
  this->all_opts_.add_options(name)
    ("r,recovery", "Reboot into recovery mode",
     cxxopts::value<bool>()->default_value("false"));

  // clang-format on
  this->_Parse(argc, argv);
}

int
ifm3d::RebootApp::Run()
{
  if (this->vm_->count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  ifm3d::Device::boot_mode mode = this->vm_->count("recovery") ?
                                    ifm3d::Device::boot_mode::RECOVERY :
                                    ifm3d::Device::boot_mode::PRODUCTIVE;

  ifm3d::SWUpdater::Ptr swupdater;
  swupdater = std::make_shared<ifm3d::SWUpdater>(this->cam_);

  if (swupdater->WaitForRecovery(-1))
    {
      swupdater->RebootToProductive();
    }
  else
    {
      this->cam_->Reboot(mode);
    }

  return 0;
}
