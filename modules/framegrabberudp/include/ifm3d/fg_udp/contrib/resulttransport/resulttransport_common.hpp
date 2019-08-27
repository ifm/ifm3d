// -*- c++ -*-
/*
 * Copyright (C) 2019 ifm electronics GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef UDPRESULTSTRUCTS_HPP
#define UDPRESULTSTRUCTS_HPP

#include <memory>

namespace ifm {

constexpr uint8_t  FRAME_HEADER[]      = { 's', 't', 'a', 'r' };
constexpr uint8_t  FRAME_FOOTER[]      = { 's', 't', 'o', 'p' };
constexpr uint32_t PACKET_MAGIC_NUMBER = static_cast<uint32_t>('#') << 0 | static_cast<uint32_t>('i') << 8
                                         | static_cast<uint32_t>('f') << 16 | static_cast<uint32_t>('m') << 24;
#pragma pack(push, 1)
enum class ChannelType : uint32_t
{
    FRAME                         = 0,    // Frame header/footer
    RADIAL_DISTANCE_IMAGE         = 100,  // Radial distance image
    NORM_AMPLITUDE_IMAGE          = 101,  // Normalized amplitude image
    CARTESIAN_X_COMPONENT         = 200,  // X-image
    CARTESIAN_Y_COMPONENT         = 201,  // Y-image
    CARTESIAN_Z_COMPONENT         = 202,  // Z-image
    CARTESIAN_ALL                 = 203,  // XYZ-Image
    UNIT_VECTOR_ALL               = 223,  // Unit vector matrix
    CONFIDENCE_IMAGE              = 300,  // Confidence image
    JSON_DIAGNOSTIC               = 305,  // JSON Diagnostic Data
    EXTRINSIC_CALIB               = 400,  // Extrinsic calibration
    INTRINSIC_CALIB               = 401,  // Intrinsic calibration
    INVERSE_INTRINSIC_CALIBRATION = 402,  // Inverse intrinsic calibration
};
#pragma pack(pop)

enum PixelFormat : uint32_t
{
    FORMAT_8U   = 0,   // 8bit unsigned integer
    FORMAT_8S   = 1,   // 8bit signed integer
    FORMAT_16U  = 2,   // 16bit unsigned integer
    FORMAT_16S  = 3,   // 16bit signed integer
    FORMAT_32U  = 4,   // 32bit unsigned integer
    FORMAT_32S  = 5,   // 32bit signed integer
    FORMAT_32F  = 6,   // 32bit floating point number
    FORMAT_64U  = 7,   // 64bit unsigned integer
    FORMAT_64F  = 8,   // 64bit floating point number
    FORMAT_16U2 = 9,   // 2 x 16bit unsigned integer
    FORMAT_32F3 = 10,  // 3 x 32bit floating point
};

#pragma pack(push, 1)
struct PacketHeader
{
    uint32_t    magic;
    uint32_t    packet_counter;
    uint32_t    frame_counter;
    uint16_t    number_of_packets_in_frame;
    uint16_t    packet_index_in_frame;
    uint16_t    number_of_packets_in_channel;
    uint16_t    index_of_packet_in_channel;
    ChannelType channel_id;
    uint32_t    total_channel_length;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ChunkHeader
{
    ChannelType channel_type;
    uint32_t    chunk_size;       // Size of the whole image chunk in bytes.
    uint32_t    header_size;      // Number of bytes starting from PIXEL_DATA
    uint32_t    header_version;   // Version number of the header (=2)
    uint32_t    image_width;      // Image width in pixel
    uint32_t    image_height;     // Image height in pixel
    PixelFormat pixel_format;     // Available Pixel formats
    uint32_t    time_stamp;       // Timestamp in μS (deprecated)
    uint32_t    frame_count;      // The overall frame count according to algorithmoutput
    uint32_t    status_code;      // This field is used to communicate errors on the deviceDevice Status
    uint32_t    time_stamp_sec;   // Timestamp seconds
    uint32_t    time_stamp_nsec;  // Timestamp nanoseconds
};
#pragma pack(pop)

struct Error
{
    enum ErrorCode
    {
        OK = 0,

        INVALID_PACKET_MAGIC,
        INVALID_PACKET_DATA,
        INVALID_CHANNEL_DATA,

        FRAME_SIZE_TOO_BIG,

        USER = 10000,
    };

    Error(int code = OK, const std::string& msg = {}, std::shared_ptr<void> userdata = nullptr)
    : code(static_cast<ErrorCode>(code))
    , msg(msg)
    , userdata(userdata)
    {
    }

    Error(const Error&) = default;

    int                   code;
    std::string           msg;
    std::shared_ptr<void> userdata;

    operator bool() { return code != OK; }
};

}

#endif  // UDPRESULTSTRUCTS_HPP
