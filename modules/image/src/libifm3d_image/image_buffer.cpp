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

#include <ifm3d/image/image_buffer.h>
#include <cstdint>
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <ifm3d/fg/byte_buffer.h>
#include <image_buffer_impl.hpp>

//--------------------------------
// ImageBuffer class
//--------------------------------

ifm3d::ImageBuffer::ImageBuffer()
  : ifm3d::ByteBuffer<ifm3d::ImageBuffer>(),
    pImpl(new ifm3d::ImageBuffer::Impl())
{ }

ifm3d::ImageBuffer::~ImageBuffer() = default;

// move ctor
ifm3d::ImageBuffer::ImageBuffer(ifm3d::ImageBuffer&& src_buff)
  : ifm3d::ImageBuffer::ImageBuffer()
{
  this->SetBytes(src_buff.bytes_, false);
}

// move assignment
ifm3d::ImageBuffer&
ifm3d::ImageBuffer::operator= (ifm3d::ImageBuffer&& src_buff)
{
  this->SetBytes(src_buff.bytes_, false);
  return *this;
}

// copy ctor
ifm3d::ImageBuffer::ImageBuffer(const ifm3d::ImageBuffer& src_buff)
  : ifm3d::ImageBuffer::ImageBuffer()
{
  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
}

// copy assignment operator
ifm3d::ImageBuffer&
ifm3d::ImageBuffer::operator= (const ifm3d::ImageBuffer& src_buff)
{
  if (this == &src_buff)
    {
      return *this;
    }

  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
  return *this;
}

cv::Mat
ifm3d::ImageBuffer::DistanceImage()
{
  this->Organize();
  return this->pImpl->DistanceImage();
}

cv::Mat
ifm3d::ImageBuffer::UnitVectors()
{
  this->Organize();
  return this->pImpl->UnitVectors();
}

cv::Mat
ifm3d::ImageBuffer::GrayImage()
{
  this->Organize();
  return this->pImpl->GrayImage();
}

cv::Mat
ifm3d::ImageBuffer::AmplitudeImage()
{
  this->Organize();
  return this->pImpl->AmplitudeImage();
}

cv::Mat
ifm3d::ImageBuffer::RawAmplitudeImage()
{
  this->Organize();
  return this->pImpl->RawAmplitudeImage();
}

cv::Mat
ifm3d::ImageBuffer::ConfidenceImage()
{
  this->Organize();
  return this->pImpl->ConfidenceImage();
}

cv::Mat
ifm3d::ImageBuffer::XYZImage()
{
  this->Organize();
  return this->pImpl->XYZImage();
}

pcl::PointCloud<ifm3d::PointT>::Ptr
ifm3d::ImageBuffer::Cloud()
{
  this->Organize();
  return this->pImpl->Cloud();
}

void
ifm3d::ImageBuffer::_ImCreate(ifm3d::image_chunk im,
                              std::uint32_t fmt,
                              std::size_t idx,
                              std::uint32_t width,
                              std::uint32_t height,
                              int nchan,
                              std::uint32_t npts,
                              const std::vector<std::uint8_t>& bytes)
{
  this->pImpl->ImCreate(im, fmt, idx, width, height, nchan, npts, bytes);
}

void
ifm3d::ImageBuffer::_CloudCreate(std::uint32_t fmt,
                                 std::size_t xidx,
                                 std::size_t yidx,
                                 std::size_t zidx,
                                 std::uint32_t width,
                                 std::uint32_t height,
                                 std::uint32_t npts,
                                 const std::vector<std::uint8_t>& bytes)
{
  this->pImpl->CloudCreate(fmt, xidx, yidx, zidx, width, height, npts, bytes);
}
