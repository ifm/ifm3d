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

#include <ifm3d/device/err.h>
#include <exception>
#include <iostream>
#include <ifm3d/tools/main_command.hpp>

int
main(int argc, char** argv)
{
  try
    {
      CLI::App app{"ifm3d Command Line Interface(CLI)"};

      ifm3d::MainCommand cli;
      cli.CreateCommand(&app);

      argv = app.ensure_utf8(argv);

      CLI11_PARSE(app, argc, argv);
    }
  catch (const ifm3d::Error& ex)
    {
      if (ex.code() == IFM3D_TOOL_COMMAND_UNSUPPORTED_DEVICE)
        {
          std::cerr << ex.what() << std::endl;
        }
      else
        {
          std::cerr << "ifm3d error: " << ex.code() << std::endl
                    << ex.what() << std::endl;
        }
      return 1;
    }
  catch (const std::exception& ex)
    {
      std::cerr << "ifm3d error: " << ex.what() << std::endl;
      return 1;
    }
  catch (...)
    {
      std::cerr << "ifm3d failed - error unknown!" << std::endl;
      return 1;
    }
  return 0;
}
