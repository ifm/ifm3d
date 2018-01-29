// -*- c++ -*-
/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 * Copyright (C) 2018 ifm syntron gmbh
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
#include <ifm3d/camera/err.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/fg/byte_buffer.h>

namespace ifm3d
{

//============================================================
// Impl interface
//============================================================
class SimpleImageBuffer::Impl
{
public:
  Impl();
  virtual ~Impl() = default;

  Img DistanceImage();
  Img UnitVectors();
  Img GrayImage();
  Img AmplitudeImage();
  Img RawAmplitudeImage();
  Img ConfidenceImage();
  Img XYZImage();
  std::vector<float> Extrinsics();
  std::vector<std::uint32_t> ExposureTimes();
  PointCloud Cloud();

  void Organize(const std::vector<std::uint8_t>& bytes);

protected:
  std::vector<float> extrinsics_;
  std::vector<std::uint32_t> exposure_times_;
  Img dist_;
  Img uvec_;
  Img gray_;
  Img amp_;
  Img ramp_;
  Img conf_;
  Img xyz_;
  PointCloud cloud_;

  template<typename T>
  void copy_data(const std::uint8_t* src, std::uint8_t* dst)
  {
#if !defined(_WIN32) && __BYTE_ORDER == __BIG_ENDIAN
    std::reverse_copy(src, src + sizeof(T), dst);
#else
    std::copy(src,  src + sizeof(T), dst);
#endif
  }

  template<typename T>
  void im_create(Img& im, std::uint32_t fmt, std::size_t idx,
                 std::uint32_t width, std::uint32_t height, int nchan,
                 std::uint32_t npts, const std::vector<std::uint8_t>& bytes)
  {
    im.data.resize(sizeof(T)*nchan*width*height);
    im.format = static_cast<pixel_format>(fmt);
    im.width = width;
    im.height = height;

    for (std::size_t i = 0; i < (npts*nchan); i++ )
      {
        const std::uint8_t* src = bytes.data() + idx + (i * sizeof(T));
        std::uint8_t* dst = im.data.data() + (i * sizeof(T));

        copy_data<T>(src, dst);
      }
  }

  template<typename T>
  void cloud_create(Img& im,
                    PointCloud& cloud,
                    std::uint32_t fmt,
                    std::size_t xidx, std::size_t yidx, std::size_t zidx,
                    std::uint32_t width, std::uint32_t height,
                    std::uint32_t npts,
                    const std::vector<std::uint8_t>& bytes)
  {
    std::size_t incr = sizeof(T);

    im.data.resize(3*npts*incr);
    im.format = static_cast<pixel_format>(fmt);
    im.width = width;
    im.height = height;

    cloud.width = width;
    cloud.height = height;
    cloud.points.resize(npts);


    T x_, y_, z_;

    // We assume, if the data from the sensor are a floating point type,
    // the data are in meters, otherwise, the sensor is sending an
    // integer type and the data are in mm.
    bool convert_to_meters = true;
    if ((im.format == pixel_format::FORMAT_32F) || (im.format == pixel_format::FORMAT_64F))
      {
        convert_to_meters = false;
      }

    for (std::size_t i = 0; i < npts;  i++, xidx += incr, yidx += incr, zidx += incr)
      {
        Point& pt = cloud.points[i];

        // convert to ifm3d coord frame
        x_ = ifm3d::mkval<T>(bytes.data()+zidx);
        y_ = -ifm3d::mkval<T>(bytes.data()+xidx);
        z_ = -ifm3d::mkval<T>(bytes.data()+yidx);

        copy_data<T>(bytes.data() + zidx, im.data.data() + 3 * i * incr);
        copy_data<T>(bytes.data() + xidx, im.data.data() + 3 * i * incr + incr);
        copy_data<T>(bytes.data() + yidx, im.data.data() + 3 * i * incr + (2 * incr));

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
ifm3d::SimpleImageBuffer::Impl::Impl()
  : extrinsics_({0.,0.,0.,0.,0.,0.}),
    exposure_times_({0,0,0})
{
}

//-------------------------------------
// Accessors
//-------------------------------------
ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::Impl::DistanceImage()
{
  return this->dist_;
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::Impl::UnitVectors()
{
  return this->uvec_;
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::Impl::GrayImage()
{
  return this->gray_;
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::Impl::AmplitudeImage()
{
  return this->amp_;
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::Impl::RawAmplitudeImage()
{
  return this->ramp_;
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::Impl::ConfidenceImage()
{
  return this->conf_;
}

ifm3d::SimpleImageBuffer::Img
ifm3d::SimpleImageBuffer::Impl::XYZImage()
{
  return this->xyz_;
}

std::vector<float>
ifm3d::SimpleImageBuffer::Impl::Extrinsics()
{
  return this->extrinsics_;
}

std::vector<std::uint32_t>
ifm3d::SimpleImageBuffer::Impl::ExposureTimes()
{
  return this->exposure_times_;
}

//-------------------------------------
// Looping over the pixel bytes
//-------------------------------------

void
ifm3d::SimpleImageBuffer::Impl::Organize(const std::vector<std::uint8_t>& bytes)
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
    (Img& im, std::uint32_t fmt, std::size_t idx)
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
          im, static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F), idx, width, height, 3, npts, bytes);
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
    (Img& im, PointCloud& cloud, std::uint32_t fmt, std::size_t xidx, std::size_t yidx, std::size_t zidx)
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

  //
  // XXX: TP March 5, 2017 -- TODO add exposure times parsing
  //
}

#endif // __IFM3D_IMAGE_IMAGE_BUFFER_IMPL_H__
