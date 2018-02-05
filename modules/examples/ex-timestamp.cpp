/*
 * Copyright (C) 2018 ifm syntron gmbh
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
// ex-timetamp.cpp
//
// Request some frames from the camera and write the timestamps to stdout


#include <iostream>
#include <memory>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <ifm3d/contrib/json.hpp>

std::string formatTimestamp(ifm3d::TimePointT timestamp) {
    using namespace std::chrono;

    std::time_t time = std::chrono::system_clock::to_time_t(
        std::chrono::time_point_cast<std::chrono::system_clock::duration>(timestamp));

    milliseconds milli = duration_cast<milliseconds>(
          timestamp.time_since_epoch() - duration_cast<seconds>(timestamp.time_since_epoch()));

    std::ostringstream s;

    s << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
      << ":" << std::setw(3) << std::setfill('0') << milli.count();

    return s.str();
}

int main(int argc, const char **argv)
{
    auto cam = ifm3d::Camera::MakeShared();

    ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
    ifm3d::FrameGrabber::Ptr fg =
      std::make_shared<ifm3d::FrameGrabber>(
        cam, ifm3d::IMG_AMP|ifm3d::IMG_CART);

    for (int i = 0; i < 10; i++) {

      if (!fg->WaitForFrame(img.get(), 1000))
        {
          std::cerr << "Error getting frame from camera" << std::endl;
          continue;
        }

      ifm3d::TimePointT timestamp = img->TimeStamp();

      std::cout << "Timestamp of frame " << std::setw(2) << std::setfill('0')
                << (i+1) << ": " << formatTimestamp(timestamp) << std::endl;
    }

    return 0;
}
