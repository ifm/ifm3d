// -*- c++ -*-
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

#ifndef __IFM3D_IMAGE_IMAGE_BUFFER_H__
#define __IFM3D_IMAGE_IMAGE_BUFFER_H__

#include <cstdint>
#include <memory>
#include <vector>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ifm3d/fg/byte_buffer.h>

namespace ifm3d
{
  using PointT = pcl::PointXYZI;
  using TimePointT = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

  /**
   * The ImageBuffer class is a composite data structure used to hold
   * time-synchronized images from the sensor. It organizes a single byte
   * buffer read from the sensor into its component parts.
   *
   * This class is not thread safe
   */
  class ImageBuffer : public ifm3d::ByteBuffer
  {
  public:
    using Ptr = std::shared_ptr<ImageBuffer>;

    /**
     * Allocates space for the individual component images.
     */
    ImageBuffer();

    /**
     * RAII decallocations
     */
    virtual ~ImageBuffer();

    // disable move semantics
    ImageBuffer(ImageBuffer&&) = delete;
    ImageBuffer& operator=(ImageBuffer&&) = delete;

    // copy semantics
    ImageBuffer(const ImageBuffer& src_buff);
    ImageBuffer& operator=(const ImageBuffer& src_buff);

    //
    // XXX: TP March 5, 2017
    //
    // Note: right now we are not documenting the shape of each returned
    // array as we may not be in position to make "absolute" statements about
    // these across all devices (e.g., data types, units of measure, etc.). For
    // now, users will have to introspect the array shapes from the data
    // structures themselves and leverage "tribal knowledge" regarding what to
    // expect across devices. Need to discuss this more with ifm.
    //

    /**
     * Accessor for the wrapped radial distance image
     */
    cv::Mat DistanceImage();

    /**
     * Accessor for the wrapped unit vectors
     */
    cv::Mat UnitVectors();

    /**
     * Accessor the the wrapped ambient light image
     */
    cv::Mat GrayImage();

    /**
     * Accessor for the normalized amplitude image
     */
    cv::Mat AmplitudeImage();

    /**
     * Accessor for the raw amplitude image
     */
    cv::Mat RawAmplitudeImage();

    /**
     * Accessor for the confidence image
     */
    cv::Mat ConfidenceImage();

    /**
     * Accessor for the OpenCV encoding of the point cloud
     *
     * 3-channel image of spatial planes X, Y, Z
     */
    cv::Mat XYZImage();

    /**
     * Returns a shared pointer to the wrapped point cloud
     */
    pcl::PointCloud<ifm3d::PointT>::Ptr Cloud();

    /**
     * Returns a 6-element vector containing the extrinsic
     * calibration of the camera. NOTE: This is the extrinsics WRT to the ifm
     * optical frame.
     *
     * The elements are: tx, ty, tz, rot_x, rot_y, rot_z
     *
     * Translation units are mm, rotations are degrees
     *
     * Users of this library are highly DISCOURAGED from using the extrinsic
     * calibration data stored on the camera itself.
     */
    std::vector<float> Extrinsics();

    /**
     * Returns a 3-element vector containing the exposure times (usec) for the
     * current frame. Unused exposure times are reported as 0.
     *
     * If all elements are reported as 0 either the exposure times are not
     * configured to be returned back in the data stream from the camera or an
     * error in parsing them has occured.
     */
    std::vector<std::uint32_t> ExposureTimes();

    /**
     * Returns the time stamp of the image data.
     *
     * NOTE: To get the timestamp of the confidence data, you
     * need to make sure your current pcic schema mask have enabled confidence data.
     */
    ifm3d::TimePointT TimeStamp();

    /**
     * Returns the temperature of the illumination unit.
     *
     * NOTE: To get the temperature of the illumination unit to the frame, you
     * need to make sure your current pcic schema asks for it.
     */
    float IlluTemp();

    /**
     * Synchronizes the parsed out image data with the internally wrapped byte
     * buffer.
     */
    virtual void Organize();

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

  }; // end: class ImageBuffer
} // end: namespace ifm3d

#endif // __IFM3D_IMAGE_IMAGE_BUFFER_H__
