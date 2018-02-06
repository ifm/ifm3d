// -*- c++ -*-
/*
 * Copyright (C) 2017 Kuhn & Völkel GmbH
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

//
// ex-pcicclient_set_io.cpp
//
// Shows how to use the PCICClient module to control the IOs state
// of the camera.
//

#include <iostream>
#include <string>
#include <thread>
#include <ifm3d/camera.h>
#include <ifm3d/pcicclient.h>


// Camera configuration string:
// * Create and activate application with index 1
// * Set LogicGraph to control IOs state via PCIC interface
const char *config = R"CONFIG(
{
    "ifm3d":
    {
        "Device":
        {
            "ActiveApplication": "1"
        },
        "Apps":
        [
            {
                "Name": "PCICClient Example",
                "Description": "Manipulates digital IOs",
                "Index" : "1",
                "LogicGraph": "{\n    \"IOMap\": {\n        \"OUT1\": \"PCIC_OUT\",\n        \"OUT2\": \"PCIC_OUT\"\n    },\n    \"blocks\": {\n        \"B00001\": {\n            \"pos\": {\n                \"x\": 262,\n                \"y\": 132\n            },\n            \"properties\": {\n            },\n            \"type\": \"PIN_EVENT_PCIC_O_CMD\"\n        },\n        \"B00003\": {\n            \"pos\": {\n                \"x\": 600,\n                \"y\": 75\n            },\n            \"properties\": {\n                \"pulse_duration\": 0\n            },\n            \"type\": \"DIGITAL_OUT1\"\n        },\n        \"B00005\": {\n            \"pos\": {\n                \"x\": 600,\n                \"y\": 200\n            },\n            \"properties\": {\n                \"pulse_duration\": 0\n            },\n            \"type\": \"DIGITAL_OUT2\"\n        }\n    },\n    \"connectors\": {\n        \"C00000\": {\n            \"dst\": \"B00003\",\n            \"dstEP\": 0,\n            \"src\": \"B00001\",\n            \"srcEP\": 0\n        },\n        \"C00001\": {\n            \"dst\": \"B00005\",\n            \"dstEP\": 0,\n            \"src\": \"B00001\",\n            \"srcEP\": 0\n        }\n    }\n}\n"
            }
        ]
    }
}
)CONFIG";

int main(int argc, char** argv)
{
  // Create camera
  ifm3d::Camera::Ptr cam = std::make_shared<ifm3d::Camera>();

  // Configure camera to allow user defined IO state
  cam->FromJSONStr(config);

  // Create pcic interface
  ifm3d::PCICClient::Ptr pcic = std::make_shared<ifm3d::PCICClient>(cam);

  // Start setting IOs (and led flashing)
  pcic->Call("o010"); // OUT1 off
  pcic->Call("o020"); // OUT2 off
  for(int i = 0; i < 10; ++i)
    {
      std::cout << "Pass " << (i+1) << "/" << 10 << std::endl;

      pcic->Call("o011"); // OUT1 on
      std::cout << "State: " << pcic->Call("O01?") << " " << pcic->Call("O02?") << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      pcic->Call("o021"); // OUT2 on
      std::cout << "State: " << pcic->Call("O01?") << " " << pcic->Call("O02?") << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      pcic->Call("o010"); // OUT1 off
      std::cout << "State: " << pcic->Call("O01?") << " " << pcic->Call("O02?") << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      pcic->Call("o020"); // OUT2 off
      std::cout << "State: " << pcic->Call("O01?") << " " << pcic->Call("O02?") << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      std::cout << std::endl;
    }

  return 0;
}
