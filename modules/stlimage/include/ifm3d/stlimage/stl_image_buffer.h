// -*- c++ -*-
/*
 * Copyright 2018-present ifm electronic, gmbh
 * Copyright 2017 Love Park Robotics, LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_STLIMAGE_IMAGE_BUFFER_H__
#define __IFM3D_STLIMAGE_IMAGE_BUFFER_H__

#include <cstdint>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <ifm3d/fg/byte_buffer.h>
#include <ifm3d/stlimage/image.h>

namespace ifm3d
{
  
  /**
   * The StlImageBuffer class is a composite data structure used to hold
   * time-synchronized images from the sensor. It organizes a single byte
   * buffer read from the sensor into its component parts.
   *
   * This class is not thread safe
   */
  class StlImageBuffer : public ifm3d::ByteBuffer<ifm3d::StlImageBuffer>
  {
  public:
    friend class ifm3d::ByteBuffer<ifm3d::StlImageBuffer>;
    using Ptr = std::shared_ptr<StlImageBuffer>;

    /**
     * Allocates space for the individual component images.
     */
    StlImageBuffer();

    /**
     * RAII deallocations
     */
    ~StlImageBuffer();

    // move semantics
    StlImageBuffer(StlImageBuffer&&);
    StlImageBuffer& operator=(StlImageBuffer&&);

    // copy ctor/assignment operator
    StlImageBuffer(const StlImageBuffer& src_buff);
    StlImageBuffer& operator=(const StlImageBuffer& src_buff);

    /**
     * Accessor for the wrapped radial distance image
     */
    ifm3d::Image DistanceImage();

    /**
     * Accessor for the wrapped unit vectors
     */
    ifm3d::Image UnitVectors();

    /**
     * Accessor the the wrapped ambient light image
     */
    ifm3d::Image GrayImage();

    /**
     * Accessor for the normalized amplitude image
     */
    ifm3d::Image AmplitudeImage();

    /**
     * Accessor for the raw amplitude image
     */
    ifm3d::Image RawAmplitudeImage();

    /**
     * Accessor for the confidence image
     */
    ifm3d::Image ConfidenceImage();

    /**
     * Accessor for the OpenCV encoding of the point cloud
     *
     * 3-channel image of spatial planes X, Y, Z
     */
    ifm3d::Image XYZImage();

  protected:
    /**
     * Hook called by the base class to populate the image containers.
     */
    template <typename T>
    void
    ImCreate(ifm3d::image_chunk im,
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
    void
    CloudCreate(std::uint32_t fmt,
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

#endif // __IFM3D_STLIMAGE_IMAGE_BUFFER_H__

