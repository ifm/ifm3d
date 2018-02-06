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
// ex-pcicclient_async_messages.cpp
//
// Shows how to use the PCICClient module to receive asynchronous
// notification (and error) messages from the camera.
//

#include <iostream>
#include <string>
#include <thread>
#include <ifm3d/camera.h>
#include <ifm3d/pcicclient.h>

// Camera configuration string:
// * Create two applications with indices 1 and 2
//   and activate application with index 2
const char *config = R"CONFIG(
{
    "ifm3d":
    {
        "Device":
        {
            "ActiveApplication": "2"
        },
        "Apps":
        [
            {
                "Name": "PCICClient Example 2a",
                "Description": "First application",
                "Index" : "1"
            },
            {
                "Name": "PCICClient Example 2b",
                "Description": "Second application",
                "Index" : "2"
            }
        ]
    }
}
)CONFIG";


int main(int argc, char** argv)
{
  // Create camera
  ifm3d::Camera::Ptr cam = std::make_shared<ifm3d::Camera>();

  // Configure two applications on the camera
//  cam->FromJSONStr(config);

  // Create pcic interface
  ifm3d::PCICClient::Ptr pcic = std::make_shared<ifm3d::PCICClient>(cam);

  // Set notification (and error) callbacks, which simply print received messages
  pcic->SetNotificationCallback([](const std::string& notification)
    {
      std::cout << "Notification: " << notification << std::endl;
    });

  pcic->SetErrorCallback([](const std::string& error)
    {
      std::cout << "Error: " << error << std::endl;
    });

  // Switch between applications (and receive notification 000500000)
  std::cout << "Switch to application 1" << std::endl;
  pcic->Call("a01");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "Switch to application 2" << std::endl;
  pcic->Call("a02");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  return 0;
}
