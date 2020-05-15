/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ifm3d/tools/fg/schema_app.h>
#include <cstdint>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <ifm3d/tools/cmdline_app.h>
#include <ifm3d/fg/schema.h>

ifm3d::SchemaApp::SchemaApp(int argc, const char **argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  this->local_opts_.add_options()
    ("mask",
     po::value<std::uint16_t>()->default_value(ifm3d::DEFAULT_SCHEMA_MASK),
     "`mask' used to generate an image acquisition schema")
    ("str",
     po::value<std::string>()->default_value("-"),
     "Mask string: e.g., 'IMG_AMP|IMG_CART'")
    ("dump",
     "Dump masking options and exit");

  po::store(po::command_line_parser(argc, argv).
            options(this->local_opts_).allow_unregistered().run(), this->vm_);
  po::notify(this->vm_);
}

int ifm3d::SchemaApp::Run()
{
  if (this->vm_.count("help"))
    {
      this->_LocalHelp();
      return 0;
    }

  if (this->vm_.count("dump"))
    {
      std::cout << "Masking options:" << std::endl
                << '\t' << "IMG_RDIS: "
                << (int) ifm3d::IMG_RDIS << std::endl
                << '\t' << "IMG_AMP:  "
                << (int) ifm3d::IMG_AMP << std::endl
                << '\t' << "IMG_RAMP: "
                << (int) ifm3d::IMG_RAMP << std::endl
                << '\t' << "IMG_CART: "
                << (int) ifm3d::IMG_CART << std::endl
                << '\t' << "IMG_UVEC: "
                << (int) ifm3d::IMG_UVEC << std::endl
                << '\t' << "EXP_TIME: "
                << (int) ifm3d::EXP_TIME << std::endl
                << '\t' << "IMG_GRAY: "
                << (int) ifm3d::IMG_GRAY << std::endl
                << '\t' << "ILLU_TEMP: "
                << (int) ifm3d::ILLU_TEMP << std::endl
                << '\t' << "INTR_CAL: "
                << (int) ifm3d::INTR_CAL << std::endl
                << '\t' << "INV_INTR_CAL: "
                << (int) ifm3d::INV_INTR_CAL << std::endl
                << '\t' << "JSON_MODEL: "
                << (int) ifm3d::JSON_MODEL << std::endl;

      return 0;
    }

  std::uint16_t mask = ifm3d::DEFAULT_SCHEMA_MASK;
  std::string mask_str;

  mask_str.assign(this->vm_["str"].as<std::string>());
  mask = this->vm_["mask"].as<std::uint16_t>();

  if (mask_str != "-")
    {
      mask = ifm3d::schema_mask_from_string(mask_str);
    }

  std::cout << "mask=" << (int) mask
            << ", str=" << mask_str
            << std::endl << "---" << std::endl
            << "PCIC (O3D-compatible): " << std::endl
            << ifm3d::make_schema(mask)
            << std::endl << "---" << std::endl
            << "XML-RPC (O3X-compatible): " << std::endl
            << ifm3d::make_o3x_json_from_mask(mask)
            << std::endl;

  return 0;
}
