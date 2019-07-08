// -*- c++ -*-
/*
 * Copyright (C) 2018 ifm electronic, gmbh
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

#ifndef __IFM3D_OPENCV_OPENCV_BUFFER_H__
#define __IFM3D_OPENCV_OPENCV_BUFFER_H__

#include <cstdint>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <ifm3d/fg/byte_buffer.h>

namespace ifm3d
{
  class OpenCVBuffer : public ifm3d::ByteBuffer<ifm3d::OpenCVBuffer>
  {
  public:
    friend class ifm3d::ByteBuffer<ifm3d::OpenCVBuffer>;
    using Ptr = std::shared_ptr<OpenCVBuffer>;

    // ctor/dtor
    OpenCVBuffer();
    ~OpenCVBuffer();

    // move semantics
    OpenCVBuffer(OpenCVBuffer&&);
    OpenCVBuffer& operator=(OpenCVBuffer&&);

    // copy semantics
    OpenCVBuffer(const OpenCVBuffer& src_buff);
    OpenCVBuffer& operator=(const OpenCVBuffer& src_buff);

    // accessors
    cv::Mat DistanceImage();
    cv::Mat UnitVectors();
    cv::Mat GrayImage();
    cv::Mat AmplitudeImage();
    cv::Mat RawAmplitudeImage();
    cv::Mat ConfidenceImage();
    cv::Mat XYZImage();

  protected:
    template <typename T>
    void ImCreate(ifm3d::image_chunk im,
                  std::uint32_t fmt,
                  std::size_t idx,
                  std::uint32_t width,
                  std::uint32_t height,
                  int nchan,
                  std::uint32_t npts,
                  const std::vector<std::uint8_t>& bytes);

    template <typename T>
    void CloudCreate(std::uint32_t fmt,
                     std::size_t xidx,
                     std::size_t yidx,
                     std::size_t zidx,
                     std::uint32_t width,
                     std::uint32_t height,
                     std::uint32_t npts,
                     const std::vector<std::uint8_t>& bytes);

  private:
    cv::Mat dist_;
    cv::Mat uvec_;
    cv::Mat gray_;
    cv::Mat amp_;
    cv::Mat ramp_;
    cv::Mat conf_;
    cv::Mat xyz_;
    cv::Mat_<std::uint8_t> bad_; // bad pixel mask

  }; // end: class OpenCVBuffer
} // end: namespace ifm3d

#include <ifm3d/opencv/detail/opencv_buffer.hpp>

#endif // __IFM3D_OPENCV_OPENCV_BUFFER_H__
