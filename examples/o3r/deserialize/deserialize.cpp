/*
 * Copyright 2021-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */
#include <iostream>
#include <chrono>
#include <thread>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg.h>
#include <ifm3d/deserialize.h>
// Namespace used for writing time "3s"
using namespace std::chrono_literals;
// Namespace used for json pointers
using namespace ifm3d::literals;
int
main()
{
    ////////////////////////////
    // Define IP and port number
    // EDIT WITH YOUR SETTINGS
    ////////////////////////////
    const auto IP = "192.168.0.69";
    const auto PORT = "port0";
    ////////////////////////////
    // Create the O3R and FrameGrabber objects
    ////////////////////////////
    auto o3r = std::make_shared<ifm3d::O3R>(IP);
    const auto PCIC_PORT = o3r->Port(PORT).pcic_port;
    auto fg = std::make_shared<ifm3d::FrameGrabber>(o3r, PCIC_PORT);

    // Define which buffer to retrieve and start the data stream
    fg->Start({ifm3d::buffer_id::RGB_INFO});

    //////////////////////////
    // Receive a frame:
    //////////////////////////
    auto future = fg->WaitForFrame();
    if (future.wait_for(3s) != std::future_status::ready)
    {
        std::cerr << "Timeout waiting for camera!" << std::endl;
        return -1;
    }
    auto frame = future.get();
    // Get the data from the relevant buffer
    auto rgb_info_buffer = frame->GetBuffer(ifm3d::buffer_id::RGB_INFO);
    fg->Stop();
    //////////////////////////
    // Extract data from the buffer
    // Using the deserializer module
    //////////////////////////
    auto rgb_info = ifm3d::RGBInfoV1::Deserialize(rgb_info_buffer);
    std::cout << "Sample of data available in the RGBInfoV1 buffer:"
              << std::endl;
    std::cout << "RGB info timestamp: "
              << rgb_info.timestamp_ns 
              << std::endl;
    std::cout << "Exposure time: "
              << rgb_info.exposure_time
              << std::endl;
    std::cout << "Intrinsic calibration model id: "
              << rgb_info.intrinsic_calibration.model_id
              << std::endl;
    std::cout << "Intrinsic calibration parameter [0]: "
              << rgb_info.intrinsic_calibration.model_parameters[0]
              << std::endl;    
    return 0;
}