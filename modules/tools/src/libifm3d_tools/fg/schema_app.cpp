/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/tools/fg/schema_app.h>
#include <cstdint>
#include <iostream>
#include <string>
#include <ifm3d/tools/cmdline_app.h>

ifm3d::SchemaApp::SchemaApp(int argc,
                            const char** argv,
                            const std::string& name)
  : ifm3d::CmdLineApp(argc, argv, name)
{
  //   // clang-format off
  //   this->all_opts_.add_options(name)
  //     ("mask",
  //      "`mask' used to generate an image acquisition schema",
  //      cxxopts::value<std::uint16_t>()->default_value(std::to_string(ifm3d::DEFAULT_SCHEMA_MASK)))
  //     ("str",
  //      "Mask string: e.g., 'IMG_AMP|IMG_CART'",
  //      cxxopts::value<std::string>()->default_value("-"))
  //     ("dump",
  //      "Dump masking options and exit");

  //   // clang-format on
  //   this->_Parse(argc, argv);
}

int
ifm3d::SchemaApp::Run()
{
  //   if (this->vm_->count("help"))
  //     {
  //       this->_LocalHelp();
  //       return 0;
  //     }

  //   if (this->vm_->count("dump"))
  //     {
  //       std::cout << "Masking options:" << std::endl
  //                 << '\t' << "IMG_RDIS: " << (int)ifm3d::IMG_RDIS <<
  //                 std::endl
  //                 << '\t' << "IMG_AMP:  " << (int)ifm3d::IMG_AMP <<
  //                 std::endl
  //                 << '\t' << "IMG_RAMP: " << (int)ifm3d::IMG_RAMP <<
  //                 std::endl
  //                 << '\t' << "IMG_CART: " << (int)ifm3d::IMG_CART <<
  //                 std::endl
  //                 << '\t' << "IMG_UVEC: " << (int)ifm3d::IMG_UVEC <<
  //                 std::endl
  //                 << '\t' << "EXP_TIME: " << (int)ifm3d::EXP_TIME <<
  //                 std::endl
  //                 << '\t' << "IMG_GRAY: " << (int)ifm3d::IMG_GRAY <<
  //                 std::endl
  //                 << '\t' << "ILLU_TEMP: " << (int)ifm3d::ILLU_TEMP <<
  //                 std::endl
  //                 << '\t' << "INTR_CAL: " << (int)ifm3d::INTR_CAL <<
  //                 std::endl
  //                 << '\t' << "INV_INTR_CAL: " << (int)ifm3d::INV_INTR_CAL
  //                 << std::endl
  //                 << '\t' << "JSON_MODEL: " << (int)ifm3d::JSON_MODEL
  //                 << std::endl;

  //       return 0;
  //     }

  //   std::uint16_t mask = ifm3d::DEFAULT_SCHEMA_MASK;
  //   std::string mask_str;

  //   mask_str.assign((*this->vm_)["str"].as<std::string>());
  //   mask = (*this->vm_)["mask"].as<std::uint16_t>();

  //   if (mask_str != "-")
  //     {
  //       mask = ifm3d::schema_mask_from_string(mask_str);
  //     }

  //   std::cout << "mask=" << (int)mask << ", str=" << mask_str << std::endl
  //             << "---" << std::endl
  //             << "PCIC (O3D-compatible): " << std::endl
  //             << ifm3d::make_schema(mask) << std::endl
  //             << "---" << std::endl
  //             << "XML-RPC (O3X-compatible): " << std::endl
  //             << ifm3d::make_o3x_json_from_mask(mask) << std::endl;

  return 0;
}
