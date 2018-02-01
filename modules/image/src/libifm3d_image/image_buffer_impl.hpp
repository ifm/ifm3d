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
    virtual ~Impl() = default;

    cv::Mat DistanceImage();
    cv::Mat UnitVectors();
    cv::Mat GrayImage();
    cv::Mat AmplitudeImage();
    cv::Mat RawAmplitudeImage();
    cv::Mat ConfidenceImage();
    cv::Mat XYZImage();
    float IlluTemp();
    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud();
    std::vector<float> Extrinsics();
    std::vector<std::uint32_t> ExposureTimes();
    ifm3d::TimePointT TimeStamp();

    void Organize(const std::vector<std::uint8_t>& bytes);

  protected:
    std::vector<float> extrinsics_;
    std::vector<std::uint32_t> exposure_times_;
    float illu_temp_;
    pcl::PointCloud<ifm3d::PointT>::Ptr cloud_;
    ifm3d::TimePointT time_stamp_;
    cv::Mat dist_;
    cv::Mat uvec_;
    cv::Mat gray_;
    cv::Mat amp_;
    cv::Mat ramp_;
    cv::Mat conf_;
    cv::Mat xyz_;

    template<typename T>
    void im_create(cv::Mat& im, std::uint32_t fmt, std::size_t idx,
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
    }

    template<typename T>
    void cloud_create(cv::Mat& im,
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
            }

          // convert to ifm3d coord frame
          x_ = ifm3d::mkval<T>(bytes.data()+zidx);
          y_ = -ifm3d::mkval<T>(bytes.data()+xidx);
          z_ = -ifm3d::mkval<T>(bytes.data()+yidx);

          xyz_ptr[xyz_col] = x_;
          xyz_ptr[xyz_col + 1] = y_;
          xyz_ptr[xyz_col + 2] = z_;

          if (convert_to_meters)
            {
              pt.x = x_ / 1000.0f;
              pt.y = y_ / 1000.0f;
              pt.z = z_ / 1000.0f;
            }
          else
            {
              pt.x = x_;
              pt.y = y_;
              pt.z = z_;
            }

          pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
          pt.intensity = 0.; // we fill this in later
        }
    }

    template<typename T>
    void set_cloud_intensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
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
  : cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
    extrinsics_({0.,0.,0.,0.,0.,0.}),
    exposure_times_({0,0,0}),
    time_stamp_(std::chrono::system_clock::now())
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

std::vector<float>
ifm3d::ImageBuffer::Impl::Extrinsics()
{
  return this->extrinsics_;
}

std::vector<std::uint32_t>
ifm3d::ImageBuffer::Impl::ExposureTimes()
{
  return this->exposure_times_;
}

ifm3d::TimePointT
ifm3d::ImageBuffer::Impl::TimeStamp()
{
  return this->time_stamp_;
}

float
ifm3d::ImageBuffer::Impl::IlluTemp()
{
  return this->illu_temp_;
}
//-------------------------------------
// Looping over the pixel bytes
//-------------------------------------

void
ifm3d::ImageBuffer::Impl::Organize(const std::vector<std::uint8_t>& bytes)
{
  // indices to the start of each chunk of interest in the buffer
  std::size_t INVALID_IDX = std::numeric_limits<std::size_t>::max();
  std::size_t xyzidx = INVALID_IDX;
  std::size_t xidx = INVALID_IDX;
  std::size_t yidx = INVALID_IDX;
  std::size_t zidx = INVALID_IDX;
  std::size_t aidx = INVALID_IDX;
  std::size_t raw_aidx = INVALID_IDX;
  std::size_t cidx = INVALID_IDX;
  std::size_t didx = INVALID_IDX;
  std::size_t uidx = INVALID_IDX;
  std::size_t extidx = INVALID_IDX;
  std::size_t gidx = INVALID_IDX;

  xyzidx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::CARTESIAN_ALL);
  if (xyzidx == INVALID_IDX)
    {
      xidx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::CARTESIAN_X);
      yidx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::CARTESIAN_Y);
      zidx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::CARTESIAN_Z);
    }
  else
    {
      xidx = ifm3d::get_chunk_index(bytes,
                                    ifm3d::image_chunk::CARTESIAN_X,
                                    xyzidx + 48);

      yidx = ifm3d::get_chunk_index(bytes,
                                    ifm3d::image_chunk::CARTESIAN_Y,
                                    xyzidx + 48);

      zidx = ifm3d::get_chunk_index(bytes,
                                    ifm3d::image_chunk::CARTESIAN_Z,
                                    xyzidx + 48);
    }
  aidx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::AMPLITUDE);
  raw_aidx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::RAW_AMPLITUDE);
  cidx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::CONFIDENCE);
  didx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::RADIAL_DISTANCE);
  uidx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::UNIT_VECTOR_ALL);
  gidx = ifm3d::get_chunk_index(bytes, ifm3d::image_chunk::GRAY);
  extidx = ifm3d::get_chunk_index(bytes,
                                  ifm3d::image_chunk::EXTRINSIC_CALIBRATION);

  VLOG(IFM3D_PROTO_DEBUG) << "xyzidx=" << xyzidx
                          << ", xidx=" << xidx
                          << ", yidx=" << yidx
                          << ", zidx=" << zidx
                          << ", aidx=" << aidx
                          << ", raw_aidx=" << raw_aidx
                          << ", cidx=" << cidx
                          << ", didx=" << didx
                          << ", uidx=" << uidx
                          << ", extidx=" << extidx
                          << ", gidx=" << gidx;

  // if we do not have a confidence image we cannot go further
  if (cidx == INVALID_IDX)
    {
      LOG(ERROR) << "No confidence image found!";
      throw ifm3d::error_t(IFM3D_IMG_CHUNK_NOT_FOUND);
    }

  const std::uint32_t header_version =  ifm3d::mkval<std::uint32_t>(bytes.data()+cidx+12);
  // for the *big* time stamp minimum header version 2 is needed
  if( header_version > 1 )
    {
      // Retrieve the timespamp information from the confidence data
      const std::uint32_t timestampSec =
          ifm3d::mkval<std::uint32_t>(bytes.data()+cidx+40);
      const std::uint32_t timestampNsec =
          ifm3d::mkval<std::uint32_t>(bytes.data()+cidx+44);
      // convert the time stamp into a TimePointT
      this->time_stamp_ = ifm3d::TimePointT {
          std::chrono::seconds{timestampSec}
          + std::chrono::nanoseconds{timestampNsec}
    };
    }
  else
    {
      // There is no *big* time stamp in chunk version 1
      this->time_stamp_ = std::chrono::system_clock::now();
    }


  bool A_OK = aidx != INVALID_IDX;
  bool RAW_A_OK = raw_aidx != INVALID_IDX;
  bool D_OK = didx != INVALID_IDX;
  bool U_OK = uidx != INVALID_IDX;
  bool EXT_OK = extidx != INVALID_IDX;
  bool G_OK = gidx != INVALID_IDX;
  bool CART_OK =
    ((xidx != INVALID_IDX) && (yidx != INVALID_IDX) && (zidx != INVALID_IDX));

  // pixel format of each image
  std::uint32_t INVALID_FMT = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t cfmt = ifm3d::mkval<std::uint32_t>(bytes.data()+cidx+24);
  std::uint32_t xfmt =
    CART_OK ?
    ifm3d::mkval<std::uint32_t>(bytes.data()+xidx+24) :
    INVALID_FMT;
  std::uint32_t yfmt =
    CART_OK ?
    ifm3d::mkval<std::uint32_t>(bytes.data()+yidx+24) :
    INVALID_FMT;
  std::uint32_t zfmt =
    CART_OK ?
    ifm3d::mkval<std::uint32_t>(bytes.data()+zidx+24) :
    INVALID_FMT;
  std::uint32_t afmt =
    A_OK ?
    ifm3d::mkval<std::uint32_t>(bytes.data()+aidx+24) :
    INVALID_FMT;
  std::uint32_t raw_afmt =
    RAW_A_OK ?
    ifm3d::mkval<std::uint32_t>(bytes.data()+raw_aidx+24) :
    INVALID_FMT;
  std::uint32_t dfmt =
    D_OK ?
    ifm3d::mkval<std::uint32_t>(bytes.data()+didx+24) :
    INVALID_FMT;
  std::uint32_t ufmt =
    U_OK ?
    ifm3d::mkval<std::uint32_t>(bytes.data()+uidx+24) :
    INVALID_FMT;
  std::uint32_t extfmt =
    EXT_OK ?
    ifm3d::mkval<std::uint32_t>(bytes.data()+extidx+24) :
    INVALID_FMT;
  std::uint32_t gfmt =
    G_OK ?
    ifm3d::mkval<std::uint32_t>(bytes.data()+gidx+24) :
    INVALID_FMT;

  VLOG(IFM3D_PROTO_DEBUG) << "xfmt=" << xfmt
                          << ", yfmt=" << yfmt
                          << ", zfmt=" << zfmt
                          << ", afmt=" << afmt
                          << ", raw_afmt=" << raw_afmt
                          << ", cfmt=" << cfmt
                          << ", dfmt=" << dfmt
                          << ", ufmt=" << ufmt
                          << ", extfmt=" << extfmt
                          << ", gfmt=" << gfmt;

  // get the image dimensions
  std::uint32_t width =
    ifm3d::mkval<std::uint32_t>(bytes.data()+cidx+16);
  std::uint32_t height =
    ifm3d::mkval<std::uint32_t>(bytes.data()+cidx+20);
  std::uint32_t npts = width * height;

  VLOG(IFM3D_PROTO_DEBUG) << "npts=" << npts
                          << ", width x height="
                          << width << " x " << height;

  auto im_wrapper =
    [this,width,height,npts,&bytes]
    (cv::Mat& im, std::uint32_t fmt, std::size_t idx)
    {
      switch (fmt)
        {
        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U):
          this->im_create<std::uint8_t>(
            im, fmt, idx, width, height, 1, npts, bytes);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S):
          this->im_create<std::int8_t>(
            im, fmt, idx, width, height, 1, npts, bytes);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U):
          this->im_create<std::uint16_t>(
            im, fmt, idx, width, height, 1, npts, bytes);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
          this->im_create<std::int16_t>(
            im, fmt, idx, width, height, 1, npts, bytes);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S):
          this->im_create<std::int32_t>(
            im, fmt, idx, width, height, 1, npts, bytes);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
          this->im_create<float>(
            im, fmt, idx, width, height, 1, npts, bytes);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3):
          this->im_create<float>(
            im, fmt, idx, width, height, 3, npts, bytes);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F):
          this->im_create<double>(
            im, fmt, idx, width, height, 1, npts, bytes);
          break;

        default:
          LOG(ERROR) << "Cannot create image with pixel format = " << fmt;
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
    };

  auto cloud_wrapper =
    [this,width,height,npts,&bytes]
    (cv::Mat& im, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
     std::uint32_t fmt, std::size_t xidx, std::size_t yidx, std::size_t zidx)
    {
      switch (fmt)
        {
        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
          this->cloud_create<std::int16_t>(im,      // OpenCV point cloud
                                           cloud,   // PCL point cloud
                                           fmt,     // format for `im`
                                           xidx,    // index of x-pix
                                           yidx,    // index of y-pix
                                           zidx,    // index of z-pix
                                           width,   // n-cols
                                           height,  // n-rows
                                           npts,    // total points
                                           bytes);  // raw bytes
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
          this->cloud_create<float>(im,      // OpenCV point cloud
                                    cloud,   // PCL point cloud
                                    fmt,     // format for `im`
                                    xidx,    // index of x-pix
                                    yidx,    // index of y-pix
                                    zidx,    // index of z-pix
                                    width,   // n-cols
                                    height,  // n-rows
                                    npts,    // total points
                                    bytes);  // raw bytes
          break;

        default:
          LOG(ERROR) << "Cannot create cloud with pixel format = " << fmt;
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
    };

  //
  // Move index pointers to where the pixel data starts and parse
  // out the 2D image data
  //
  std::uint32_t pixel_data_offset =
    ifm3d::mkval<std::uint32_t>(bytes.data()+cidx+8);

  cidx += pixel_data_offset;
  im_wrapper(this->conf_, cfmt, cidx);

  if (D_OK)
    {
      didx += pixel_data_offset;
      im_wrapper(this->dist_, dfmt, didx);
    }

  if (U_OK)
    {
      uidx += pixel_data_offset;
      im_wrapper(this->uvec_, ufmt, uidx);
    }

  if (G_OK)
    {
      gidx += pixel_data_offset;
      im_wrapper(this->gray_, gfmt, gidx);
    }

  if (A_OK)
    {
      aidx += pixel_data_offset;
      im_wrapper(this->amp_, afmt, aidx);
    }

  if (RAW_A_OK)
    {
      raw_aidx += pixel_data_offset;
      im_wrapper(this->ramp_, raw_afmt, raw_aidx);
    }

  //
  // point cloud construction
  //
  if (CART_OK)
    {
      xidx += pixel_data_offset;
      yidx += pixel_data_offset;
      zidx += pixel_data_offset;
      cloud_wrapper(this->xyz_, this->cloud_, xfmt, xidx, yidx, zidx);

      // Here is a hook to color the intensity channel by default
      // we use the amplitude, but, I could see using the gray image,
      // raw amplitude, etc. Maybe a new env var to control this?
      if (A_OK)
        {
          std::uint8_t depth = this->amp_.type() & CV_MAT_DEPTH_MASK;
          switch (depth)
            {
            case CV_8U:
              this->set_cloud_intensity<std::uint8_t>(this->cloud_, this->amp_);
              break;

            case CV_8S:
              this->set_cloud_intensity<std::int8_t>(this->cloud_, this->amp_);
              break;

            case CV_16U:
              this->set_cloud_intensity<std::uint16_t>(this->cloud_,
                                                       this->amp_);
              break;

            case CV_16S:
              this->set_cloud_intensity<std::int16_t>(this->cloud_, this->amp_);
              break;

            case CV_32S:
              this->set_cloud_intensity<std::int32_t>(this->cloud_, this->amp_);
              break;

            case CV_32F:
              this->set_cloud_intensity<float>(this->cloud_, this->amp_);
              break;

            case CV_64F:
              this->set_cloud_intensity<double>(this->cloud_, this->amp_);
              break;

            default:
              break;
            }
        }
    }

  //
  // extrinsic calibration
  //
  if (EXT_OK)
    {
      extidx += pixel_data_offset;
      if (extfmt !=
          static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F))
        {
          LOG(ERROR) << "Extrinsics are expected to be float32, not: "
                     << extfmt;
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }

      for (std::size_t i = 0; i < 6; ++i, extidx += 4)
        {
          this->extrinsics_[i] = ifm3d::mkval<float>(bytes.data()+extidx);
        }
    }

  // OK, now we want to see if the temp illu and exposure times are present,
  // if they are, we want to parse them out and store them in the image buffer.
  // Since the extrinsics are invariant and should *always* be present, we use
  // the current index of the extrinsics.
  if (EXT_OK)
    {
      std::size_t extime_idx = extidx;
      size_t bytes_left = bytes.size() - extime_idx;

      // Read extime (6 bytes string + 3x 4 bytes uint32_t)
      if (bytes_left >= 18
          && std::equal(bytes.begin() + extidx,
                        bytes.begin() + extidx + 6,
                        std::begin("extime")))
        {
          extime_idx += 6;
          bytes_left -= 6;

          // 3 exposure times
          for (std::size_t i = 0; i < 3; ++i)
            {
              if ((bytes_left - 6) <= 0)
                {
                  break;
                }

              std::uint32_t extime2 =
                  ifm3d::mkval<std::uint32_t>(
                    bytes.data() + extime_idx);

              this->exposure_times_.at(i) = extime2;

              extime_idx += 4;
              bytes_left -= 4;
            }
        }
      else
        {
          std::fill(this->exposure_times_.begin(),
                    this->exposure_times_.end(), 0);
        }

      // Read temp_illu (9 bytes string + 4 bytes float)
      if (bytes_left >= 13
          && std::equal(bytes.begin() + extidx,
                        bytes.begin() + extidx + 8,
                        std::begin("temp_illu")))
        {
          extime_idx += 9;
          bytes_left -= 9;

          this->illu_temp_ =
              ifm3d::mkval<float>(bytes.data() + extime_idx);

          extime_idx += 4;
          bytes_left -= 4;

          DLOG(INFO) << "IlluTemp= " << this->illu_temp_;
        }
      else
        {
          this->illu_temp_ = 0;
        }
    }
  else
    {
      LOG(WARNING) << "Checking for illu temp and exposure times skipped (cant trust extidx)";
    }
}

#endif // __IFM3D_IMAGE_IMAGE_BUFFER_IMPL_H__
