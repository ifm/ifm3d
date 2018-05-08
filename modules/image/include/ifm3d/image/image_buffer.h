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
  class ImageBuffer : public ifm3d::ByteBuffer<ifm3d::ImageBuffer>
  {
  public:
    friend class ifm3d::ByteBuffer<ifm3d::ImageBuffer>;
    using Ptr = std::shared_ptr<ImageBuffer>;

    /**
     * Allocates space for the individual component images.
     */
    ImageBuffer();

    /**
     * RAII deallocations
     */
    ~ImageBuffer();

    // move semantics
    ImageBuffer(ImageBuffer&&);
    ImageBuffer& operator=(ImageBuffer&&);

    // copy ctor/assignment operator
    ImageBuffer(const ImageBuffer& src_buff);
    ImageBuffer& operator=(const ImageBuffer& src_buff);

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

  protected:
    /**
     * Hook called by the base class to populate the image containers.
     */
    template <typename T>
    void ImCreate(ifm3d::image_chunk im,
                  std::uint32_t fmt,
                  std::size_t idx,
                  std::uint32_t width,
                  std::uint32_t height,
                  int nchan,
                  std::uint32_t npts,
                  const std::vector<std::uint8_t>& bytes)
    {
      // NOTE: we drop the template parameter here (and re-establish it later)
      // so that we can maintain our pimpl abstraction in support of the
      // general user-base. It is understood that this has a non-zero cost
      // associated with it. Other higher-peformance image containers are
      // currently being contemplated for advanced / latency sensitive users.
      this->_ImCreate(im, fmt, idx, width, height, nchan, npts, bytes);
    }

    /**
     * Hook called by the base class to populate the point cloud containers.
     */
    template <typename T>
    void CloudCreate(std::uint32_t fmt,
                     std::size_t xidx,
                     std::size_t yidx,
                     std::size_t zidx,
                     std::uint32_t width,
                     std::uint32_t height,
                     std::uint32_t npts,
                     const std::vector<std::uint8_t>& bytes)
  {
    // See "NOTE" in `ImCreate` as to why we are dropping the template
    // parameter here. Same rationale applies.
    this->_CloudCreate(fmt, xidx, yidx, zidx, width, height, npts, bytes);
  }

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    /**
     * Wrapper called by the main `ImCreate` callback hook that allows us to
     * proxy the image container population into our wrapped `Impl`.
     */
    void _ImCreate(ifm3d::image_chunk im,
                   std::uint32_t fmt,
                   std::size_t idx,
                   std::uint32_t width,
                   std::uint32_t height,
                   int nchan,
                   std::uint32_t npts,
                   const std::vector<std::uint8_t>& bytes);

    /**
     * Wrapper called by the main `CloudCreate` callback hook that allows us to
     * proxy the cloud container population into our wrapped `Impl`.
     */
    void _CloudCreate(std::uint32_t fmt,
                      std::size_t xidx,
                      std::size_t yidx,
                      std::size_t zidx,
                      std::uint32_t width,
                      std::uint32_t height,
                      std::uint32_t npts,
                      const std::vector<std::uint8_t>& bytes);

  }; // end: class ImageBuffer
} // end: namespace ifm3d

#endif // __IFM3D_IMAGE_IMAGE_BUFFER_H__
