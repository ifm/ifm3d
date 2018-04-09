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
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ifm3d/fg/byte_buffer.h>

namespace ifm3d
{
  using PointT = pcl::PointXYZI;

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
