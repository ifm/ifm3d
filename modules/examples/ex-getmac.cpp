/*
 * Copyright (C) 2016 Love Park Robotics, LLC
 * Copyright (C) 2017 ifm syntron gmbh
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
// ex-getmac.cpp
//
// Request the MAC address from the camera. The MAC address can be used as
// a unique identifier.


#include <iostream>
#include <memory>
#include <ifm3d/camera.h>
#include <ifm3d/contrib/json.hpp>

int main(int argc, const char **argv)
{
    auto cam = ifm3d::Camera::MakeShared();

    // get the JSON configuration data from the camera
    auto jsonConfig = cam->ToJSON();
    // extract the Net Object from the JSON data
    auto netConfig = jsonConfig["ifm3d"]["Net"];
    // print out the MAC address
    std::cout << "The MAC address of the camera: " << netConfig["MACAddress"] << std::endl;
    return 0;
}
