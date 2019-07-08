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

#ifndef __IFM3D_IMAGE_IMAGE_BUFFER_IMPL_H__
#define __IFM3D_IMAGE_IMAGE_BUFFER_IMPL_H__

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/fg/byte_buffer.h>

namespace ifm3d
{
  std::unordered_map<std::uint32_t, int> PIX_LUT
  {
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U), CV_8U},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S), CV_8S},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U), CV_16U},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S), CV_16S},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S), CV_32S},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), CV_32F},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3), CV_32F},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F), CV_64F}
  };

  std::unordered_map<std::uint32_t, int> PIX_LUT3
  {
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U), CV_8UC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S), CV_8SC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U), CV_16UC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S), CV_16SC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S), CV_32SC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), CV_32FC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3), CV_32FC3},
    {static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F), CV_64FC3}
  };

  std::unordered_map<int, std::size_t> PIX_SZ
  {
    {CV_8U, 1},
    {CV_8S, 1},
    {CV_16U, 2},
    {CV_16S, 2},
    {CV_32S, 4},
    {CV_32F, 4},
    {CV_64F, 8}
  };

  //============================================================
  // Impl interface
  //============================================================
  class ImageBuffer::Impl
  {
  public:
    Impl();
    ~Impl() = default;

    cv::Mat DistanceImage();
    cv::Mat UnitVectors();
    cv::Mat GrayImage();
    cv::Mat AmplitudeImage();
    cv::Mat RawAmplitudeImage();
    cv::Mat ConfidenceImage();
    cv::Mat XYZImage();
    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud();

    void ImCreate(ifm3d::image_chunk im,
                  std::uint32_t fmt,
                  std::size_t idx,
                  std::uint32_t width,
                  std::uint32_t height,
                  int nchan,
                  std::uint32_t npts,
                  const std::vector<std::uint8_t>& bytes);

    void CloudCreate(std::uint32_t fmt,
                     std::size_t xidx,
                     std::size_t yidx,
                     std::size_t zidx,
                     std::uint32_t width,
                     std::uint32_t height,
                     std::uint32_t npts,
                     const std::vector<std::uint8_t>& bytes);

  protected:
    pcl::PointCloud<ifm3d::PointT>::Ptr cloud_;
    cv::Mat dist_;
    cv::Mat uvec_;
    cv::Mat gray_;
    cv::Mat amp_;
    cv::Mat ramp_;
    cv::Mat conf_;
    cv::Mat xyz_;
    cv::Mat_<std::uint8_t> bad_; // mask of bad pixels

    template<typename T>
    void _ImCreate(cv::Mat& im, ifm3d::image_chunk chunk,
                   std::uint32_t fmt, std::size_t idx,
                   std::uint32_t width, std::uint32_t height, int nchan,
                   std::uint32_t npts, const std::vector<std::uint8_t>& bytes)
    {
      std::size_t incr = sizeof(T) * nchan;
      if (nchan == 3)
        {
          im.create(height, width, ifm3d::PIX_LUT3.at(fmt));
        }
      else
        {
          im.create(height, width, ifm3d::PIX_LUT.at(fmt));
        }

      int col = 0;
      int row = -1;
      int col3 = 0;

      T* ptr;

      for (std::size_t i = 0; i < npts; ++i, idx += incr)
        {
          col = i % width;
          col3 = col * 3;

          if (col == 0)
            {
              row += 1;
              ptr = im.ptr<T>(row);
            }

          if (nchan == 3)
            {
              ptr[col3] = ifm3d::mkval<T>(bytes.data()+idx);
              ptr[col3 + 1] = ifm3d::mkval<T>(bytes.data()+idx+sizeof(T));
              ptr[col3 + 2] = ifm3d::mkval<T>(bytes.data()+idx+(sizeof(T)*2));
            }
          else
            {
              ptr[col] = ifm3d::mkval<T>(bytes.data()+idx);
            }
        }

      //
      // There are only certain image types we want to flag bad
      // pixels for. For example we do *not* want to flag the confidence image
      // or the unit vectors. To that end, we do not do this in-line above
      // with a bunch of nasty if-statements for each pixel. Instead,
      // we take another pass over the image. This vectorized approach
      // should be fast/cache-friendly enough to not have us introduce too
      // much additional latency.
      //
      // constexpr T bad_pixel =
      //   std::numeric_limits<T>::has_quiet_NaN
      //   ? std::numeric_limits<T>::quiet_NaN() : 0;
      constexpr T bad_pixel = 0;

      if ((chunk != ifm3d::image_chunk::CONFIDENCE) &&
          (chunk != ifm3d::image_chunk::UNIT_VECTOR_ALL))
        {
          im.setTo(bad_pixel, this->bad_);
        }
    }

    template<typename T>
    void _CloudCreate(cv::Mat& im,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                      std::uint32_t fmt,
                      std::size_t xidx, std::size_t yidx, std::size_t zidx,
                      std::uint32_t width, std::uint32_t height,
                      std::uint32_t npts,
                      const std::vector<std::uint8_t>& bytes)
    {
      std::size_t incr = sizeof(T);
      im.create(height, width, ifm3d::PIX_LUT3.at(fmt));

      cloud->header.frame_id = "ifm3d";
      cloud->width = width;
      cloud->height = height;
      cloud->is_dense = true;
      cloud->points.resize(npts);

      int col = 0;
      int row = -1;
      int xyz_col = 0;

      T* xyz_ptr;
      T x_, y_, z_;
      std::uint8_t* bad_ptr;

      // constexpr T bad_pixel =
      //   std::numeric_limits<T>::has_quiet_NaN
      //   ? std::numeric_limits<T>::quiet_NaN() : 0;
      constexpr T bad_pixel = 0;

      // We assume, if the data from the sensor are a floating point type,
      // the data are in meters, otherwise, the sensor is sending an
      // integer type and the data are in mm.
      bool convert_to_meters = true;
      if ((im.type() == CV_32FC3) || (im.type() == CV_64FC3))
        {
          convert_to_meters = false;
        }

      for (std::size_t i = 0; i < npts;
           ++i, xidx += incr, yidx += incr, zidx += incr)
        {
          pcl::PointXYZI& pt = cloud->points[i];

          col = i % width;
          xyz_col = col * 3;
          if (col == 0)
            {
              row += 1;
              xyz_ptr = im.ptr<T>(row);
              bad_ptr = this->bad_.ptr(row);
            }

          // convert to ifm3d coord frame
          x_ = ifm3d::mkval<T>(bytes.data()+zidx);
          y_ = -ifm3d::mkval<T>(bytes.data()+xidx);
          z_ = -ifm3d::mkval<T>(bytes.data()+yidx);

          if (bad_ptr[col] == 0)
            {
              xyz_ptr[xyz_col] = x_;
              xyz_ptr[xyz_col + 1] = y_;
              xyz_ptr[xyz_col + 2] = z_;
            }
          else
            {
              xyz_ptr[xyz_col] = bad_pixel;
              xyz_ptr[xyz_col + 1] = bad_pixel;
              xyz_ptr[xyz_col + 2] = bad_pixel;
            }

          if (convert_to_meters)
            {
              if (bad_ptr[col] == 0)
                {
                  pt.x = x_ / 1000.0f;
                  pt.y = y_ / 1000.0f;
                  pt.z = z_ / 1000.0f;
                }
              else
                {
                  pt.x = bad_pixel;
                  pt.y = bad_pixel;
                  pt.z = bad_pixel;
                }
            }
          else
            {
              if (bad_ptr[col] == 0)
                {
                  pt.x = x_;
                  pt.y = y_;
                  pt.z = z_;
                }
              else
                {
                  pt.x = bad_pixel;
                  pt.y = bad_pixel;
                  pt.z = bad_pixel;
                }
            }

          pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
          pt.intensity = 0;
        }
    }

    template<typename T>
    void _SetCloudIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            const cv::Mat& im)
    {
      int numpts = im.rows * im.cols;
      if (cloud->points.size() != numpts)
        {
          VLOG(IFM3D_PROTO_DEBUG)
            << "Shape mismatch when coloring pcl intensity: "
            << cloud->points.size() << " vs "
            << numpts;
          return;
        }

      const T* row_ptr;
      int col = 0;
      int row = -1;

      for (std::size_t i = 0; i < numpts; ++i)
        {
          pcl::PointXYZI& pt = cloud_->points[i];

          col = i % im.cols;
          if (col == 0)
            {
              row += 1;
              row_ptr = im.ptr<T>(row);
            }

          pt.intensity = static_cast<float>(row_ptr[col]);
        }
    }
  }; // end: class ImageBuffer::Impl

} // end: namespace ifm3d

//============================================================
// Impl -- Implementation Details
//============================================================

//-------------------------------------
// ctor/dtor
//-------------------------------------
ifm3d::ImageBuffer::Impl::Impl()
  : cloud_(new pcl::PointCloud<pcl::PointXYZI>())
{
  this->cloud_->sensor_origin_.setZero();
  this->cloud_->sensor_orientation_.w() = 1.0f;
  this->cloud_->sensor_orientation_.x() = 0.0f;
  this->cloud_->sensor_orientation_.y() = 0.0f;
  this->cloud_->sensor_orientation_.z() = 0.0f;
}

//-------------------------------------
// Accessors
//-------------------------------------
cv::Mat
ifm3d::ImageBuffer::Impl::DistanceImage()
{
  return this->dist_;
}

cv::Mat
ifm3d::ImageBuffer::Impl::UnitVectors()
{
  return this->uvec_;
}

cv::Mat
ifm3d::ImageBuffer::Impl::GrayImage()
{
  return this->gray_;
}

cv::Mat
ifm3d::ImageBuffer::Impl::AmplitudeImage()
{
  return this->amp_;
}

cv::Mat
ifm3d::ImageBuffer::Impl::RawAmplitudeImage()
{
  return this->ramp_;
}

cv::Mat
ifm3d::ImageBuffer::Impl::ConfidenceImage()
{
  return this->conf_;
}

cv::Mat
ifm3d::ImageBuffer::Impl::XYZImage()
{
  return this->xyz_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
ifm3d::ImageBuffer::Impl::Cloud()
{
  return this->cloud_;
}

void
ifm3d::ImageBuffer::Impl::ImCreate(ifm3d::image_chunk im,
                                   std::uint32_t fmt,
                                   std::size_t idx,
                                   std::uint32_t width,
                                   std::uint32_t height,
                                   int nchan,
                                   std::uint32_t npts,
                                   const std::vector<std::uint8_t>& bytes)
{
  cv::Mat *image;
  switch (im)
    {
    case ifm3d::image_chunk::CONFIDENCE:
      image = &this->conf_;
      break;

    case ifm3d::image_chunk::AMPLITUDE:
      image = &this->amp_;
      break;

    case ifm3d::image_chunk::RADIAL_DISTANCE:
      image = &this->dist_;
      break;

    case ifm3d::image_chunk::UNIT_VECTOR_ALL:
      image = &this->uvec_;
      break;

    case ifm3d::image_chunk::RAW_AMPLITUDE:
      image = &this->ramp_;
      break;

    case ifm3d::image_chunk::GRAY:
      image = &this->gray_;
      break;

    default:
      return;
    }

  switch (fmt)
    {
    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U):
      this->_ImCreate<std::uint8_t>(
        *image, im, fmt, idx, width, height, 1, npts, bytes);
      break;

    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S):
      this->_ImCreate<std::int8_t>(
        *image, im, fmt, idx, width, height, 1, npts, bytes);
      break;

    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U):
      this->_ImCreate<std::uint16_t>(
        *image, im, fmt, idx, width, height, 1, npts, bytes);
      break;

    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
      this->_ImCreate<std::int16_t>(
        *image, im, fmt, idx, width, height, 1, npts, bytes);
      break;

    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S):
      this->_ImCreate<std::int32_t>(
        *image, im, fmt, idx, width, height, 1, npts, bytes);
      break;

    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
      this->_ImCreate<float>(
        *image, im, fmt, idx, width, height, 1, npts, bytes);
      break;

    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3):
      this->_ImCreate<float>(
        *image, im, fmt, idx, width, height, 3, npts, bytes);
      break;

    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F):
      this->_ImCreate<double>(
        *image, im, fmt, idx, width, height, 1, npts, bytes);
      break;

    default:
      LOG(ERROR) << "Unknown image pixel format: " << fmt;
      throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
    }

  // update the bad pixel mask if we just saw the confidence image
  if (im == ifm3d::image_chunk::CONFIDENCE)
    {
      cv::bitwise_and(this->conf_, 0x1, this->bad_);
    }
}

void
ifm3d::ImageBuffer::Impl::CloudCreate(std::uint32_t fmt,
                                      std::size_t xidx,
                                      std::size_t yidx,
                                      std::size_t zidx,
                                      std::uint32_t width,
                                      std::uint32_t height,
                                      std::uint32_t npts,
                                      const std::vector<std::uint8_t>& bytes)
{
  switch (fmt)
    {
    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
      this->_CloudCreate<std::int16_t>(this->xyz_,
                                       this->cloud_,
                                       fmt,
                                       xidx,
                                       yidx,
                                       zidx,
                                       width,
                                       height,
                                       npts,
                                       bytes);
      break;

    case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
      this->_CloudCreate<float>(this->xyz_,
                                this->cloud_,
                                fmt,
                                xidx,
                                yidx,
                                zidx,
                                width,
                                height,
                                npts,
                                bytes);
      break;

    default:
      LOG(ERROR) << "Unknown cloud pixel format: " << fmt;
      throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
    }

  std::uint8_t depth = this->amp_.type() & CV_MAT_DEPTH_MASK;
  switch (depth)
    {
    case CV_8U:
      this->_SetCloudIntensity<std::uint8_t>(this->cloud_, this->amp_);
      break;

    case CV_8S:
      this->_SetCloudIntensity<std::int8_t>(this->cloud_, this->amp_);
      break;

    case CV_16U:
      this->_SetCloudIntensity<std::uint16_t>(this->cloud_,
                                               this->amp_);
      break;

    case CV_16S:
      this->_SetCloudIntensity<std::int16_t>(this->cloud_, this->amp_);
      break;

    case CV_32S:
      this->_SetCloudIntensity<std::int32_t>(this->cloud_, this->amp_);
      break;

    case CV_32F:
      this->_SetCloudIntensity<float>(this->cloud_, this->amp_);
      break;

    case CV_64F:
      this->_SetCloudIntensity<double>(this->cloud_, this->amp_);
      break;

    default:
      break;
    }
}

#endif // __IFM3D_IMAGE_IMAGE_BUFFER_IMPL_H__
