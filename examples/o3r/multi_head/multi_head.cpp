/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */


/*How to: receive data from multiple heads
 One feature of the O3R platform is to enable the use of multiple camera heads of different types (2D, 3D, various resolutions, etc). 
 In this example, we show how to retrieve the pcic port number for each head connected to the VPU along with its type and create `FrameGrabber` and `ImageBuffer` objects for each.
*/
#include <ifm3d/camera/camera_o3r.h>
#include <iostream>
#include <iomanip>
#include <ifm3d/fg.h>
#include <ifm3d/stlimage.h>

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
    auto cam = std::make_shared<ifm3d::O3RCamera>();
    // Retreive ports configuration 
    json conf = cam->Get();
    // Initialize the structures
    std::vector<ifm3d::FrameGrabber::Ptr> fgs;
    auto im =  std::make_shared<ifm3d::StlImageBuffer>(); 

    std::cout << "Available connections:" << std::endl;

    // Retrieve all the ports numbers for display later.
    std::vector<std::string> port_nbs;
    for (auto port_nb : conf["ports"].items())
    {
        port_nbs.push_back(port_nb.key());
    }

    int j = 0;
    for (const auto& port : conf["ports"])
    {
        // Create lists of connected PCIC ports along with types
        const auto pcic = port["/data/pcicTCPPort"_json_pointer];
        const auto type = port["/info/features/type"_json_pointer];
        //Display connected port with type
        std::cout << "Port: " << port_nbs[j] << "\t PCIC: " << pcic << "\t Type: " << type << std::endl;
        // Create list of FrameGrabber and ImageBuffer objects for connected ports    
        auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::DEFAULT_SCHEMA_MASK, pcic);
        fgs.push_back(fg);    
        j++;
    }

    // Grab frames from each heads
    int i = 0;
    // The timestamp has two parts, the timestamp in seconds and the timestamp in nanoseconds
    ifm3d::TimePointT timestamp;
    for (auto fg : fgs)
    {
        if (! fg->WaitForFrame(im.get(), 3000))
        {
            std::cerr << "Timeout waiting for camera!" << std::endl;
            return -1;
        }

        i++;
        timestamp = im->TimeStamp();
        std::cout << "Timestamp of frame "
                << std::setw(2) << std::setfill('0')
                << ": " << formatTimestamp(timestamp)
                << std::endl;  
    }

    return 0;
}