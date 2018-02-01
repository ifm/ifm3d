/*
 * Copyright (C) 2016 Love Park Robotics, LLC
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
// ex-exposure_times.cpp
//
//  Shows how to change imager exposure times on the fly while streaming in
//  pixel data and validating the setting of the exposure times registered to
//  the frame data.
//

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <ifm3d/camera.h>
#include "ifm3d/fg.h"
#include <ifm3d/image.h>

int main(int argc, const char **argv)
{

  // example configuration for the camera we will use for exemplary purpose
  // we will use a double exposure imager.
  std::string json = R"(
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
                "TriggerMode": "1",
                "Index": "1",
                "Imager":
                {
                    "ExposureTime": "5000",
                    "ExposureTimeList": "125;5000",
                    "ExposureTimeRatio": "40",
                    "Type":"under5m_moderate"
                }
              }
           ]
          }
        }
      )";


  // instantiate the camera and set the configuration
  ifm3d::Camera::Ptr cam = std::make_shared<ifm3d::Camera>();
  std::cout << "Setting camera configuration: " << std::endl
            << json << std::endl;


  // create our image buffer to hold frame data from the camera
  ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();

  // instantiate our framegrabber and be sure to explicitly tell it to
  // stream back the exposure times registered to the frame data
  ifm3d::FrameGrabber::Ptr fg =
    std::make_shared<ifm3d::FrameGrabber>(
      cam, ifm3d::DEFAULT_SCHEMA_MASK|ifm3d::EXP_TIME|ifm3d::ILLU_TEMP);

  // a vector to hold the exposure times (we will just print them to the
  // screen)
  std::vector<std::uint32_t> exposure_times;

  // a map use to modulate the `ExposureTime` and `ExposureTimeRatio`
  // on-the-fly. We seed it with data consistent with our config above
  std::unordered_map<std::string, std::string> params =
    {
      {"imager_001/ExposureTime", "5000"},
      {"imager_001/ExposureTimeRatio", "40"}
    };

  // create a session with the camera so we can modulate the exposure times
  cam->RequestSession();

  // set our session timeout --
  //
  // NOTE: I'm going to do nothing with this here. However, in a *real*
  // application, you will have to send `Heartbeats` at least every `hb_secs`
  // seconds to the camera. The best technique for doing that is left as an
  // exercise for the reader.
  int hb_secs = cam->Heartbeat(300);

  // now we start looping over the image data, every 20 frames, we will
  // change the exposure times, after 100 frames we will exit.
  int i = 0;
  while (true)
    {
      if (! fg->WaitForFrame(img.get(), 1000))
        {
          std::cerr << "Timeout waiting for camera!" << std::endl;
          continue;
        }

      // get the exposure times registered to the frame data
      exposure_times = img->ExposureTimes();
      auto illu_temp = img->IlluTemp();

      // depending on your imager config, you can have up to 3 exposure
      // times. I'll print all three for exemplary purposes, but we know there
      // are only two valid ones based on our double exposure imager
      // configuration from above. We expect the third to be 0.
      std::cout << "Exposure Time 0: " << exposure_times.at(0)
                << std::endl;
      std::cout << "Exposure Time 1: " << exposure_times.at(1)
                << std::endl;
      std::cout << "Exposure Time 2: " << exposure_times.at(2)
                << std::endl;
      std::cout << "Illu Temp: " << illu_temp
                << std::endl;
      std::cout << "---" << std::endl;

      i++;

      if (i == 100)
        {
          break;
        }

      if (i % 20 == 0)
        {
          std::cout << "Setting long exposure time to: ";
          if (exposure_times.at(1) == 5000)
            {
              std::cout << 10000 << std::endl;
              params["imager_001/ExposureTime"] = "10000";
            }
          else
            {
              std::cout << 5000 << std::endl;
              params["imager_001/ExposureTime"] = "5000";
            }

          cam->SetTemporaryApplicationParameters(params);
        }
    }

  //
  // In a long-running program, you will need to take care to
  // clean up your session if necessary. Here we don't worry about it because
  // the camera dtor will do that for us.
  //
  std::cout << "et voila." << std::endl;
  return 0;
}
