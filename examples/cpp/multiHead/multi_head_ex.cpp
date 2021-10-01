/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */


/*How to: receive data from multiple heads
 One feature of the O3R platform is to enable the use of multiple camera heads of different types (2D, 3D, various resolutions, etc). 
 In this example, we show how to retrieve the pcic port number for each head connected to the VPU along with its type and create `FrameGrabber` and `ImageBuffer` objects for each.
*/
#include <opencv2/core/core.hpp>
#include <ifm3d/camera/camera_o3r.h>
#include <iostream>
#include <iomanip>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>


// This function formats the timestamps for proper display
// a.k.a converts to local time
std::string formatTimestamp(ifm3d::TimePointT timestamp)
{
    using namespace std::chrono;
    std::time_t time = std::chrono::system_clock::to_time_t(
        std::chrono::time_point_cast<std::chrono::system_clock::duration>(
          timestamp));

    milliseconds milli = duration_cast<milliseconds>(
          timestamp.time_since_epoch() - duration_cast<seconds>(
            timestamp.time_since_epoch()));

    std::ostringstream s;
    s << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
      << ":" << std::setw(3) << std::setfill('0') << milli.count();

    return s.str();
}


int main(){

    // Declare the camera object
    auto cam = ifm3d::CameraBase::MakeShared();
    // Retreive ports configuration 
    json conf = cam->ToJSON();
    auto ports = conf["ports"];
    // Initialize the structures
    std::vector<int> pcic_ports;
    std::vector<std::string> types;
    std::vector<ifm3d::FrameGrabber::Ptr> fgs;
    std::vector<ifm3d::ImageBuffer::Ptr> ims;

    std::cout << "Available connections:" << std::endl;

    for (const auto& port : ports.items())
    {
        // Create lists of connected PCIC ports along with types
        auto pcic = port.value()["data"]["pcicTCPPort"];
        auto type = port.value()["info"]["features"]["type"];
        pcic_ports.push_back(pcic);
        types.push_back(type);    
        //Display connected port with type
        std::cout << "Port: " << port.key() << "\t PCIC: " << pcic << "\t Type: " << type << std::endl;
        // Create list of FrameGrabber and ImageBuffer objects for connected ports    
        auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, 10, pcic);
        fgs.push_back(fg);
        auto im = std::make_shared<ifm3d::ImageBuffer>();
        ims.push_back(im);

    }

    // Grab frames from each heads
    int i = 0;
    // The timestamp has two parts, the timestamp in seconds and the timestamp in nanoseconds
    ifm3d::TimePointT timestamp;
    for (auto fg : fgs)
    {
        auto im = ims[i];
        fg->WaitForFrame(im.get(), 3000);
        i++;
        timestamp = im->TimeStamp();
        std::cout << "Timestamp of frame "
                  << std::setw(2) << std::setfill('0')
                  << ": " << formatTimestamp(timestamp)
                  << std::endl;    
    }

    return 0;
}