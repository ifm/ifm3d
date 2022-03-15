// -*- c++ -*-
/*
 * Copyright 2018 ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __IFM3D_FG_DETAIL_BYTE_BUFFER_HPP__
#define __IFM3D_FG_DETAIL_BYTE_BUFFER_HPP__

#include <ifm3d/camera/logging.h>
#include <ifm3d/camera/err.h>
#include <ifm3d/fg/distance_image_info.h>
#include <cstring>

//-------------------------------------
// The ByteBuffer<Dervied> class impl
//-------------------------------------

// ctor
template <typename Derived>
ifm3d::ByteBuffer<Derived>::ByteBuffer()
  : dirty_(false),
    extrinsics_({0., 0., 0., 0., 0., 0.}),
    intrinsics_(
      {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}),
    inverseIntrinsics_(
      {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}),
    intrinsic_available(false),
    inverse_intrinsic_available(false),
    exposure_times_({0, 0, 0}),
    time_stamps_({}),
    json_model_("{}")
{}

// dtor
template <typename Derived>
ifm3d::ByteBuffer<Derived>::~ByteBuffer() = default;

// move ctor
template <typename Derived>
ifm3d::ByteBuffer<Derived>::ByteBuffer(ifm3d::ByteBuffer<Derived>&& src_buff)
  : ifm3d::ByteBuffer<Derived>::ByteBuffer()
{
  this->SetBytes(src_buff.bytes_, false);
}

// move assignment
template <typename Derived>
ifm3d::ByteBuffer<Derived>&
ifm3d::ByteBuffer<Derived>::operator=(ifm3d::ByteBuffer<Derived>&& src_buff)
{
  this->SetBytes(src_buff.bytes_, false);
  return *this;
}

// copy ctor
template <typename Derived>
ifm3d::ByteBuffer<Derived>::ByteBuffer(
  const ifm3d::ByteBuffer<Derived>& src_buff)
  : ifm3d::ByteBuffer<Derived>::ByteBuffer()
{
  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
}

// copy assignment operator
template <typename Derived>
ifm3d::ByteBuffer<Derived>&
ifm3d::ByteBuffer<Derived>::operator=(
  const ifm3d::ByteBuffer<Derived>& src_buff)
{
  if (this == &src_buff)
    {
      return *this;
    }

  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
  return *this;
}

template <typename Derived>
void
ifm3d::ByteBuffer<Derived>::SetBytes(std::vector<std::uint8_t>& buff,
                                     bool copy)
{
  if (copy)
    {
      std::size_t sz = buff.size();
      this->bytes_.resize(sz);

      std::copy(buff.begin(), buff.begin() + sz, this->bytes_.begin());
    }
  else
    {
      buff.swap(this->bytes_);
    }

  this->_SetDirty(true);
}

template <typename Derived>
void
ifm3d::ByteBuffer<Derived>::_SetDirty(bool flg) noexcept
{
  this->dirty_ = flg;
}

template <typename Derived>
bool
ifm3d::ByteBuffer<Derived>::Dirty() const noexcept
{
  return this->dirty_;
}

template <typename Derived>
std::vector<std::uint8_t>
ifm3d::ByteBuffer<Derived>::Bytes()
{
  return this->bytes_;
}

template <typename Derived>
std::vector<float>
ifm3d::ByteBuffer<Derived>::Extrinsics()
{
  this->Organize();
  return this->extrinsics_;
}

template <typename Derived>
std::vector<float>
ifm3d::ByteBuffer<Derived>::Intrinsics()
{
  this->Organize();
  return this->intrinsics_;
}

template <typename Derived>
std::vector<float>
ifm3d::ByteBuffer<Derived>::InverseIntrinsics()
{
  this->Organize();
  return this->inverseIntrinsics_;
}

template <typename Derived>
std::vector<std::uint32_t>
ifm3d::ByteBuffer<Derived>::ExposureTimes()
{
  this->Organize();
  return this->exposure_times_;
}

template <typename Derived>
ifm3d::TimePointT
ifm3d::ByteBuffer<Derived>::TimeStamp()
{
  this->Organize();
  return this->time_stamps_[0];
}

template <typename Derived>
std::vector<ifm3d::TimePointT>
ifm3d::ByteBuffer<Derived>::TimeStamps()
{
  this->Organize();
  return this->time_stamps_;
}

template <typename Derived>
float
ifm3d::ByteBuffer<Derived>::IlluTemp()
{
  this->Organize();
  return this->illu_temp_;
}

template <typename Derived>
std::string
ifm3d::ByteBuffer<Derived>::JSONModel()
{
  this->Organize();
  return this->json_model_;
}

template <typename Derived>
void
ifm3d::ByteBuffer<Derived>::Organize()
{
  if (!this->Dirty())
    {
      return;
    }

  // indices to the start of each chunk of interest in the buffer
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
  std::size_t intridx = INVALID_IDX;
  std::size_t invintridx = INVALID_IDX;
  std::size_t jsonidx = INVALID_IDX;
  std::size_t jpegidx = INVALID_IDX;
  std::size_t dist_noiseidx = INVALID_IDX;

  xyzidx =
    ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::CARTESIAN_ALL);
  if (xyzidx == INVALID_IDX)
    {
      xidx =
        ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::CARTESIAN_X);

      yidx =
        ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::CARTESIAN_Y);

      zidx =
        ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::CARTESIAN_Z);
    }
  else
    {
      xidx = ifm3d::get_chunk_index(this->bytes_,
                                    ifm3d::image_chunk::CARTESIAN_X,
                                    xyzidx + 48);

      yidx = ifm3d::get_chunk_index(this->bytes_,
                                    ifm3d::image_chunk::CARTESIAN_Y,
                                    xyzidx + 48);

      zidx = ifm3d::get_chunk_index(this->bytes_,
                                    ifm3d::image_chunk::CARTESIAN_Z,
                                    xyzidx + 48);
    }

  aidx = ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::AMPLITUDE);
  raw_aidx =
    ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::RAW_AMPLITUDE);
  cidx = ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::CONFIDENCE);
  didx =
    ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::RADIAL_DISTANCE);
  uidx =
    ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::UNIT_VECTOR_ALL);
  gidx = ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::GRAY);
  extidx = ifm3d::get_chunk_index(this->bytes_,
                                  ifm3d::image_chunk::EXTRINSIC_CALIBRATION);
  jsonidx =
    ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::JSON_MODEL);
  jpegidx = ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::JPEG);
  dist_noiseidx =
    ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::DISTANCE_NOISE);

  // As parameter will not change so only grabed and stored
  // for the first time
  if (!intrinsic_available)
    {
      intridx =
        ifm3d::get_chunk_index(this->bytes_,
                               ifm3d::image_chunk::INTRINSIC_CALIBRATION);
    }

  if (!inverse_intrinsic_available)
    {
      invintridx = ifm3d::get_chunk_index(
        this->bytes_,
        ifm3d::image_chunk::INVERSE_INTRINSIC_CALIBRATION);
    }

  // to get the metadata we use the confidence image for 3d and
  // the jpeg image for 2d
  std::size_t metaidx = cidx != INVALID_IDX ? cidx : jpegidx;

  // if we do not have a confidence or jpeg image we cannot go further
  if (metaidx == INVALID_IDX)
    {
      throw ifm3d::error_t(IFM3D_IMG_CHUNK_NOT_FOUND);
    }

  const std::uint32_t header_version =
    ifm3d::mkval<std::uint32_t>(this->bytes_.data() + metaidx + 12);
  // for the *big* time stamp minimum header version 2 is needed
  this->time_stamps_.resize(0);
  if (header_version > 1)
    {
      // Retrieve the timespamp information from the confidence or jpeg data
      const std::uint32_t timestampSec =
        ifm3d::mkval<std::uint32_t>(this->bytes_.data() + metaidx + 40);
      const std::uint32_t timestampNsec =
        ifm3d::mkval<std::uint32_t>(this->bytes_.data() + metaidx + 44);
      // convert the time stamp into a TimePointT
      this->time_stamps_.push_back(
        ifm3d::TimePointT{std::chrono::seconds{timestampSec} +
                          std::chrono::nanoseconds{timestampNsec}});
      // O3X device provides an offeset in Usec releative to timestamp
      // calculated as time_stamp_[0]
      const std::uint32_t ethernetTimeinUsecOffset =
        ifm3d::mkval<std::uint32_t>(this->bytes_.data() + cidx + 28);
      this->time_stamps_.push_back(ifm3d::TimePointT{
        this->time_stamps_[0] +
        std::chrono::microseconds{ethernetTimeinUsecOffset}});
    }
  else
    {
      // There is no *big* time stamp in chunk version 1
      this->time_stamps_.push_back(std::chrono::system_clock::now());
    }

  bool C_OK = (cidx != INVALID_IDX);
  bool A_OK = (aidx != INVALID_IDX);
  bool RAW_A_OK = (raw_aidx != INVALID_IDX);
  bool D_OK = (didx != INVALID_IDX);
  bool U_OK = (uidx != INVALID_IDX);
  bool EXT_OK = (extidx != INVALID_IDX);
  bool INTR_OK = (intridx != INVALID_IDX);
  bool INVINTR_OK = (invintridx != INVALID_IDX);
  bool G_OK = (gidx != INVALID_IDX);
  bool CART_OK =
    ((xidx != INVALID_IDX) && (yidx != INVALID_IDX) && (zidx != INVALID_IDX));
  bool JSON_OK = (jsonidx != INVALID_IDX);
  bool JPEG_OK = (jpegidx != INVALID_IDX);
  bool DIST_NOISE_OK = (dist_noiseidx != INVALID_IDX);

  // pixel format of each image
  std::uint32_t INVALID_FMT = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t cfmt =
    ifm3d::mkval<std::uint32_t>(this->bytes_.data() + cidx + 24);
  std::uint32_t xfmt =
    CART_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + xidx + 24) :
              INVALID_FMT;
  std::uint32_t yfmt =
    CART_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + yidx + 24) :
              INVALID_FMT;
  std::uint32_t zfmt =
    CART_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + zidx + 24) :
              INVALID_FMT;
  std::uint32_t afmt =
    A_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + aidx + 24) :
           INVALID_FMT;
  std::uint32_t raw_afmt =
    RAW_A_OK ?
      ifm3d::mkval<std::uint32_t>(this->bytes_.data() + raw_aidx + 24) :
      INVALID_FMT;
  std::uint32_t dfmt =
    D_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + didx + 24) :
           INVALID_FMT;
  std::uint32_t ufmt =
    U_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + uidx + 24) :
           INVALID_FMT;
  std::uint32_t extfmt =
    EXT_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + extidx + 24) :
             INVALID_FMT;
  std::uint32_t gfmt =
    G_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + gidx + 24) :
           INVALID_FMT;
  std::uint32_t intrfmt =
    INTR_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + intridx + 24) :
              INVALID_FMT;
  std::uint32_t invintrfmt =
    INVINTR_OK ?
      ifm3d::mkval<std::uint32_t>(this->bytes_.data() + invintridx + 24) :
      INVALID_FMT;
  std::uint32_t jpegfmt =
    JPEG_OK ? ifm3d::mkval<std::uint32_t>(this->bytes_.data() + jpegidx + 24) :
              INVALID_FMT;
  std::uint32_t dist_noisefmt =
    DIST_NOISE_OK ?
      ifm3d::mkval<std::uint32_t>(this->bytes_.data() + dist_noiseidx + 24) :
      INVALID_FMT;

  // get the image dimensions
  std::uint32_t width =
    ifm3d::mkval<std::uint32_t>(this->bytes_.data() + metaidx + 16);
  std::uint32_t height =
    ifm3d::mkval<std::uint32_t>(this->bytes_.data() + metaidx + 20);
  std::uint32_t npts = width * height;

  auto im_wrapper = [this, width, height, npts](ifm3d::image_chunk im,
                                                std::uint32_t fmt,
                                                std::size_t idx) {
    switch (fmt)
      {
      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U):
        this->ImCreate<std::uint8_t>(im,
                                     fmt,
                                     idx,
                                     width,
                                     height,
                                     1,
                                     npts,
                                     this->bytes_);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S):
        this->ImCreate<std::int8_t>(im,
                                    fmt,
                                    idx,
                                    width,
                                    height,
                                    1,
                                    npts,
                                    this->bytes_);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U):
        this->ImCreate<std::uint16_t>(im,
                                      fmt,
                                      idx,
                                      width,
                                      height,
                                      1,
                                      npts,
                                      this->bytes_);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
        this->ImCreate<std::int16_t>(im,
                                     fmt,
                                     idx,
                                     width,
                                     height,
                                     1,
                                     npts,
                                     this->bytes_);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S):
        this->ImCreate<std::int32_t>(im,
                                     fmt,
                                     idx,
                                     width,
                                     height,
                                     1,
                                     npts,
                                     this->bytes_);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
        this->ImCreate<float>(im,
                              fmt,
                              idx,
                              width,
                              height,
                              1,
                              npts,
                              this->bytes_);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3):
        this->ImCreate<float>(im,
                              fmt,
                              idx,
                              width,
                              height,
                              3,
                              npts,
                              this->bytes_);
        break;

      case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F):
        this->ImCreate<double>(im,
                               fmt,
                               idx,
                               width,
                               height,
                               1,
                               npts,
                               this->bytes_);
        break;

      default:
        throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
      }
  };

  auto cloud_wrapper =
    [this, width, height, npts](std::uint32_t fmt,
                                std::size_t xidx,
                                std::size_t yidx,
                                std::size_t zidx,
                                const std::vector<std::uint8_t>& bytes) {
      switch (fmt)
        {
        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
          this->CloudCreate<std::int16_t>(fmt,    // pixel format
                                          xidx,   // index of x-pix
                                          yidx,   // index of y-pix
                                          zidx,   // index of z-pix
                                          width,  // n-cols
                                          height, // n-rows
                                          npts,   // total points
                                          bytes); // raw bytes
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
          this->CloudCreate<float>(fmt,    // pixel format
                                   xidx,   // index of x-pix
                                   yidx,   // index of y-pix
                                   zidx,   // index of z-pix
                                   width,  // n-cols
                                   height, // n-rows
                                   npts,   // total points
                                   bytes); // raw bytes
          break;

        default:
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
    };

  // for an O3R device, a distance_image_info object will be created
  // for others a nullptr is returned
  auto distance_image_info =
    CreateDistanceImageInfo(this->bytes_, didx, aidx, width, height);

  //
  // Move index pointers to where the pixel data starts and parse
  // out the 2D image data
  //
  std::uint32_t pixel_data_offset =
    ifm3d::mkval<std::uint32_t>(this->bytes_.data() + metaidx + 8);

  if (C_OK)
    {
      cidx += pixel_data_offset;
      im_wrapper(ifm3d::image_chunk::CONFIDENCE, cfmt, cidx);
    }

  if (D_OK && distance_image_info == nullptr)
    {
      didx += pixel_data_offset;
      im_wrapper(ifm3d::image_chunk::RADIAL_DISTANCE, dfmt, didx);
    }

  if (U_OK)
    {
      uidx += pixel_data_offset;
      im_wrapper(ifm3d::image_chunk::UNIT_VECTOR_ALL, ufmt, uidx);
    }

  if (G_OK)
    {
      gidx += pixel_data_offset;
      im_wrapper(ifm3d::image_chunk::GRAY, gfmt, gidx);
    }

  if (A_OK && distance_image_info != nullptr) // O3R device
    {
      // the amplitude vector is derived from
      // the distance image info data
      std::vector<std::uint8_t> ampl_bytes =
        distance_image_info->getAmplitudeVector();
      this->ImCreate<float>(
        ifm3d::image_chunk::AMPLITUDE,
        static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F),
        0,
        width,
        height,
        1,
        npts,
        ampl_bytes);
    }
  else if (A_OK)
    {
      aidx += pixel_data_offset;
      im_wrapper(ifm3d::image_chunk::AMPLITUDE, afmt, aidx);
    }

  if (RAW_A_OK)
    {
      raw_aidx += pixel_data_offset;
      im_wrapper(ifm3d::image_chunk::RAW_AMPLITUDE, raw_afmt, raw_aidx);
    }

  //
  // point cloud construction
  //
  if (CART_OK)
    {
      xidx += pixel_data_offset;
      yidx += pixel_data_offset;
      zidx += pixel_data_offset;
      cloud_wrapper(xfmt, xidx, yidx, zidx, this->bytes_);
    }
  else if (D_OK && distance_image_info != nullptr) // O3R device
    {
      // the x,y,z and distance matrices are derived from
      // the distance image info data
      std::vector<std::uint8_t> xyzd_bytes =
        distance_image_info->getXYZDVector();

      // create distance image
      this->ImCreate<float>(
        ifm3d::image_chunk::RADIAL_DISTANCE,
        static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F),
        npts * 3 * FLOAT_DATA_SIZE,
        width,
        height,
        1,
        npts,
        xyzd_bytes);

      // create point cloud
      cloud_wrapper(
        static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F),
        0,
        npts * FLOAT_DATA_SIZE,
        npts * 2 * FLOAT_DATA_SIZE,
        xyzd_bytes);
    }

  //
  // intrinsic calibration
  //
  if (INTR_OK)
    {
      // size of the chunk data
      std::uint32_t chunk_size =
        ifm3d::mkval<uint32_t>(this->bytes_.data() + intridx + 4);
      intridx += pixel_data_offset;
      if (intrfmt !=
          static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F))
        {
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
      if (header_version < 2 &&
          (chunk_size - pixel_data_offset) !=
            ifm3d::NUM_INTRINSIC_PARAM * UINT32_DATA_SIZE)
        {
          throw ifm3d::error_t(IFM3D_HEADER_VERSION_MISMATCH);
        }
      for (std::size_t i = 0; i < ifm3d::NUM_INTRINSIC_PARAM;
           ++i, intridx += 4)
        {
          this->intrinsics_[i] =
            ifm3d::mkval<float>(this->bytes_.data() + intridx);
        }
      this->intrinsic_available = true;
    }

  //
  //   invert intrinsic calibration
  //
  if (INVINTR_OK)
    {
      // size of the chunk data
      std::uint32_t chunk_size =
        ifm3d::mkval<uint32_t>(this->bytes_.data() + invintridx + 4);
      invintridx += pixel_data_offset;
      if (invintrfmt !=
          static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F))
        {
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
      if (header_version < 2 &&
          (chunk_size - pixel_data_offset) !=
            ifm3d::NUM_INTRINSIC_PARAM * UINT32_DATA_SIZE)
        {
          throw ifm3d::error_t(IFM3D_HEADER_VERSION_MISMATCH);
        }
      for (std::size_t i = 0; i < ifm3d::NUM_INTRINSIC_PARAM;
           ++i, invintridx += 4)
        {
          this->inverseIntrinsics_[i] =
            ifm3d::mkval<float>(this->bytes_.data() + invintridx);
        }
      this->inverse_intrinsic_available = true;
    }

  if (JSON_OK)
    {
      std::uint32_t chunk_size =
        ifm3d::mkval<uint32_t>(this->bytes_.data() + jsonidx + 4);
      jsonidx += pixel_data_offset; // this is actually header size
      this->json_model_.resize(chunk_size - pixel_data_offset);
      std::memcpy((void*)this->json_model_.data(),
                  (void*)(this->bytes_.data() + jsonidx),
                  chunk_size - pixel_data_offset);
    }

  if (JPEG_OK)
    {
      std::uint32_t chunk_size =
        ifm3d::mkval<uint32_t>(this->bytes_.data() + jpegidx + 4);
      jpegidx += pixel_data_offset;
      if (jpegfmt !=
          static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32U))
        {
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
      this->ImCreate<std::uint32_t>(ifm3d::image_chunk::JPEG,
                                    jpegfmt,
                                    jpegidx,
                                    width,
                                    height,
                                    1,
                                    chunk_size - pixel_data_offset,
                                    this->bytes_);
    }
  if (DIST_NOISE_OK)
    {
      dist_noiseidx += pixel_data_offset;
      im_wrapper(ifm3d::image_chunk::DISTANCE_NOISE,
                 dist_noisefmt,
                 dist_noiseidx);
    }

  //
  // extrinsic calibration
  //
  if (EXT_OK)
    {
      // size of the chunk data
      std::uint32_t chunk_size =
        ifm3d::mkval<uint32_t>(this->bytes_.data() + extidx + 4);
      extidx += pixel_data_offset;
      if (extfmt !=
          static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F))
        {
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
      if (header_version < 2 &&
          (chunk_size - pixel_data_offset) !=
            ifm3d::NUM_EXTRINSIC_PARAM * UINT32_DATA_SIZE)
        {
          throw ifm3d::error_t(IFM3D_HEADER_VERSION_MISMATCH);
        }
      for (std::size_t i = 0; i < ifm3d::NUM_EXTRINSIC_PARAM; ++i, extidx += 4)
        {
          this->extrinsics_[i] =
            ifm3d::mkval<float>(this->bytes_.data() + extidx);
        }
    }
  else if (distance_image_info != nullptr) // O3R device
    {
      // renamed to extrinsic_optic_to_user in O3R
      this->extrinsics_ = distance_image_info->getExtrinsicOpticToUser();

      auto intrinsicCalibration =
        distance_image_info->getIntrinsicCalibration();
      auto inverseIntrinsicCalibration =
        distance_image_info->getInverseIntrinsicCalibration();

      this->intrinsics_ =
        std::vector<float>(std::begin(intrinsicCalibration.model_parameters),
                           std::end(intrinsicCalibration.model_parameters));

      this->inverseIntrinsics_ = std::vector<float>(
        std::begin(inverseIntrinsicCalibration.model_parameters),
        std::end(inverseIntrinsicCalibration.model_parameters));

      auto timestamps_nsec = distance_image_info->getTimestamps();
      this->time_stamps_.resize(0);

      for (auto& val : timestamps_nsec)
        {
          this->time_stamps_.push_back(
            ifm3d::TimePointT{std::chrono::nanoseconds{val}});
        }

      auto exposure_times = distance_image_info->getExposureTimes();
      this->exposure_times_.resize(0);
      for (auto& val : exposure_times)
        {
          auto ms = std::chrono::duration_cast<
            std::chrono::duration<uint32_t, std::micro>>(
            std::chrono::duration<float>(val));

          this->exposure_times_.push_back(ms.count());
        }
    }
  // OK, now we want to see if the temp illu and exposure times are present,
  // if they are, we want to parse them out and store them registered to the
  // frame data. Since the extrinsics are invariant and should *always* be
  // present, we use the current index of the extrinsics.
  if (EXT_OK)
    {
      std::size_t extime_idx = extidx;
      size_t bytes_left = this->bytes_.size() - extime_idx;

      // Read extime (6 bytes string + 3x 4 bytes uint32_t)
      if ((bytes_left >= 18) && std::equal(this->bytes_.begin() + extidx,
                                           this->bytes_.begin() + extidx + 6,
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
                ifm3d::mkval<std::uint32_t>(this->bytes_.data() + extime_idx);

              this->exposure_times_.at(i) = extime2;

              extime_idx += 4;
              bytes_left -= 4;
            }
        }
      else
        {
          std::fill(this->exposure_times_.begin(),
                    this->exposure_times_.end(),
                    0);
        }

      // Read temp_illu (9 bytes string + 4 bytes float)
      if ((bytes_left >= 13) && std::equal(this->bytes_.begin() + extidx,
                                           this->bytes_.begin() + extidx + 8,
                                           std::begin("temp_illu")))
        {
          extime_idx += 9;
          bytes_left -= 9;

          this->illu_temp_ =
            ifm3d::mkval<float>(this->bytes_.data() + extime_idx);

          extime_idx += 4;
          bytes_left -= 4;

        }
      else
        {
          this->illu_temp_ = 0;
        }
    }

  this->_SetDirty(false);
}

#endif // __IFM3D_FG_DETAIL_BYTE_BUFFER_HPP__
