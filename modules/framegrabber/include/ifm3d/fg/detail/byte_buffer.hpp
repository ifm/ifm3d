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

#ifndef __IFM3D_FG_DETAIL_BYTE_BUFFER_HPP__
#define __IFM3D_FG_DETAIL_BYTE_BUFFER_HPP__

#include <glog/logging.h>
#include <ifm3d/camera/logging.h>
#include <ifm3d/camera/err.h>
#include <cstring>

//-------------------------------------
// The ByteBuffer<Dervied> class impl
//-------------------------------------

// ctor
template <typename Derived>
ifm3d::ByteBuffer<Derived>::ByteBuffer()
  : dirty_(false),
    extrinsics_({0.,0.,0.,0.,0.,0.}),
    intrinsics_({0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.}),
    inverseIntrinsics_({0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.}),
    intrinsic_available(false),
    inverse_intrinsic_available(false),
    exposure_times_({0,0,0}),
    time_stamp_(std::chrono::system_clock::now()),
    json_model_("{}")
{ }

// dtor
template <typename Derived>
ifm3d::ByteBuffer<Derived>::~ByteBuffer() = default;

// move ctor
template <typename Derived>
ifm3d::ByteBuffer<Derived>::ByteBuffer(
  ifm3d::ByteBuffer<Derived>&& src_buff)
  : ifm3d::ByteBuffer<Derived>::ByteBuffer()
{
  this->SetBytes(src_buff.bytes_, false);
}

// move assignment
template <typename Derived>
ifm3d::ByteBuffer<Derived>&
ifm3d::ByteBuffer<Derived>::operator= (
  ifm3d::ByteBuffer<Derived>&& src_buff)
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
ifm3d::ByteBuffer<Derived>::operator= (
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

      std::copy(buff.begin(),
                buff.begin() + sz,
                this->bytes_.begin());
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
  return this->time_stamp_;
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
  if (! this->Dirty())
    {
      return;
    }

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
  std::size_t intridx = INVALID_IDX;
  std::size_t invintridx = INVALID_IDX;
  std::size_t jsonidx = INVALID_IDX;

  xyzidx = ifm3d::get_chunk_index(this->bytes_,
                                  ifm3d::image_chunk::CARTESIAN_ALL);
  if (xyzidx == INVALID_IDX)
    {
      xidx = ifm3d::get_chunk_index(this->bytes_,
                                    ifm3d::image_chunk::CARTESIAN_X);

      yidx = ifm3d::get_chunk_index(this->bytes_,
                                    ifm3d::image_chunk::CARTESIAN_Y);

      zidx = ifm3d::get_chunk_index(this->bytes_,
                                    ifm3d::image_chunk::CARTESIAN_Z);
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
  raw_aidx = ifm3d::get_chunk_index(this->bytes_,
                                    ifm3d::image_chunk::RAW_AMPLITUDE);
  cidx = ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::CONFIDENCE);
  didx = ifm3d::get_chunk_index(this->bytes_,
                                ifm3d::image_chunk::RADIAL_DISTANCE);
  uidx = ifm3d::get_chunk_index(this->bytes_,
                                ifm3d::image_chunk::UNIT_VECTOR_ALL);
  gidx = ifm3d::get_chunk_index(this->bytes_, ifm3d::image_chunk::GRAY);
  extidx = ifm3d::get_chunk_index(this->bytes_,
                                  ifm3d::image_chunk::EXTRINSIC_CALIBRATION);
  jsonidx= ifm3d::get_chunk_index(this->bytes_,
                                  ifm3d::image_chunk::JSON_MODEL);

  // As parameter will not change so only grabed and stored
  // for the first time
  if (!intrinsic_available)
    {
      intridx = ifm3d::get_chunk_index(this->bytes_,
                                      ifm3d::image_chunk::INTRINSIC_CALIBRATION);
    }

  if( !inverse_intrinsic_available )
  {
    invintridx = ifm3d::get_chunk_index(this->bytes_,
                                  ifm3d::image_chunk::INVERSE_INTRINSIC_CALIBRATION);
  }


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
                          << ", gidx=" << gidx
                          << ", intridx=" << intridx
                          << ", invintridx=" << invintridx;

  // if we do not have a confidence image we cannot go further
  if (cidx == INVALID_IDX)
    {
      LOG(ERROR) << "No confidence image found!";
      throw ifm3d::error_t(IFM3D_IMG_CHUNK_NOT_FOUND);
    }

  const std::uint32_t header_version =
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+cidx+12);
  // for the *big* time stamp minimum header version 2 is needed
  if (header_version > 1)
    {
      // Retrieve the timespamp information from the confidence data
      const std::uint32_t timestampSec =
          ifm3d::mkval<std::uint32_t>(this->bytes_.data()+cidx+40);
      const std::uint32_t timestampNsec =
          ifm3d::mkval<std::uint32_t>(this->bytes_.data()+cidx+44);
      // convert the time stamp into a TimePointT
      this->time_stamp_ =
        ifm3d::TimePointT{std::chrono::seconds{timestampSec} +
                          std::chrono::nanoseconds{timestampNsec}};
    }
  else
    {
      // There is no *big* time stamp in chunk version 1
      this->time_stamp_ = std::chrono::system_clock::now();
    }

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

  // pixel format of each image
  std::uint32_t INVALID_FMT = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t cfmt = ifm3d::mkval<std::uint32_t>(this->bytes_.data()+cidx+24);
  std::uint32_t xfmt =
    CART_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+xidx+24) :
    INVALID_FMT;
  std::uint32_t yfmt =
    CART_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+yidx+24) :
    INVALID_FMT;
  std::uint32_t zfmt =
    CART_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+zidx+24) :
    INVALID_FMT;
  std::uint32_t afmt =
    A_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+aidx+24) :
    INVALID_FMT;
  std::uint32_t raw_afmt =
    RAW_A_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+raw_aidx+24) :
    INVALID_FMT;
  std::uint32_t dfmt =
    D_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+didx+24) :
    INVALID_FMT;
  std::uint32_t ufmt =
    U_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+uidx+24) :
    INVALID_FMT;
  std::uint32_t extfmt =
    EXT_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+extidx+24) :
    INVALID_FMT;
  std::uint32_t gfmt =
    G_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+gidx+24) :
    INVALID_FMT;
  std::uint32_t intrfmt =
    INTR_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+intridx+24) :
    INVALID_FMT;
  std::uint32_t invintrfmt =
    INVINTR_OK ?
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+invintridx+24) :
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
                          << ", gfmt=" << gfmt
                          << ", intrfmt= " << intrfmt
                          << ", invintrfmt= " << invintrfmt;

  // get the image dimensions
  std::uint32_t width =
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+cidx+16);
  std::uint32_t height =
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+cidx+20);
  std::uint32_t npts = width * height;

  VLOG(IFM3D_PROTO_DEBUG) << "npts=" << npts
                          << ", width x height="
                          << width << " x " << height;

  auto im_wrapper =
    [this, width, height, npts]
    (ifm3d::image_chunk im, std::uint32_t fmt, std::size_t idx)
    {
      switch (fmt)
        {
        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8U):
          this->ImCreate<std::uint8_t>(
            im, fmt, idx, width, height, 1, npts, this->bytes_);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_8S):
          this->ImCreate<std::int8_t>(
            im, fmt, idx, width, height, 1, npts, this->bytes_);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16U):
          this->ImCreate<std::uint16_t>(
            im, fmt, idx, width, height, 1, npts, this->bytes_);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
          this->ImCreate<std::int16_t>(
            im, fmt, idx, width, height, 1, npts, this->bytes_);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32S):
          this->ImCreate<std::int32_t>(
            im, fmt, idx, width, height, 1, npts, this->bytes_);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
          this->ImCreate<float>(
            im, fmt, idx, width, height, 1, npts, this->bytes_);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F3):
          this->ImCreate<float>(
            im, fmt, idx, width, height, 3, npts, this->bytes_);
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_64F):
          this->ImCreate<double>(
            im, fmt, idx, width, height, 1, npts, this->bytes_);
          break;

        default:
          LOG(ERROR) << "Cannot create image with pixel format = " << fmt;
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
    };

  auto cloud_wrapper =
    [this, width, height, npts]
    (std::uint32_t fmt, std::size_t xidx, std::size_t yidx, std::size_t zidx)
    {
      switch (fmt)
        {
        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_16S):
          this->CloudCreate<std::int16_t>(fmt,            // pixel format
                                          xidx,           // index of x-pix
                                          yidx,           // index of y-pix
                                          zidx,           // index of z-pix
                                          width,          // n-cols
                                          height,         // n-rows
                                          npts,           // total points
                                          this->bytes_);  // raw bytes
          break;

        case static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F):
          this->CloudCreate<float>(fmt,            // pixel format
                                   xidx,           // index of x-pix
                                   yidx,           // index of y-pix
                                   zidx,           // index of z-pix
                                   width,          // n-cols
                                   height,         // n-rows
                                   npts,           // total points
                                   this->bytes_);  // raw bytes
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
    ifm3d::mkval<std::uint32_t>(this->bytes_.data()+cidx+8);

  cidx += pixel_data_offset;
  im_wrapper(ifm3d::image_chunk::CONFIDENCE, cfmt, cidx);

  if (D_OK)
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

  if (A_OK)
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
      cloud_wrapper(xfmt, xidx, yidx, zidx);
    }
  //
  // intrinsic calibration
  //
  if (INTR_OK)
    {
      //size of the chunk data
      std::uint32_t chunk_size =
      ifm3d::mkval<uint32_t >(this->bytes_.data() + intridx + 4);
      intridx += pixel_data_offset;
      if (intrfmt !=
          static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F))
        {
          LOG(ERROR) << "Intrinsic are expected to be float, not: "
                     << intrfmt;
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
      if (header_version < 2 && (chunk_size - pixel_data_offset)
          != ifm3d::NUM_INTRINSIC_PARAM * sizeof(uint32_t))
        {
          LOG(ERROR) << "Header Version expected value is >=2, not :"
                     << header_version
                     << "Intrinsic param dataLength expected value 64, not :"
                     << chunk_size - pixel_data_offset;

        throw ifm3d::error_t(IFM3D_HEADER_VERSION_MISMATCH);
        }
      for (std::size_t i = 0; i < ifm3d::NUM_INTRINSIC_PARAM; ++i, intridx += 4)
        {
          this->intrinsics_[i] =
                ifm3d::mkval<float>(this->bytes_.data()+intridx);
        }
      this->intrinsic_available = true;
    }

  //
  //   intrinsic calibration
  //
  if (INVINTR_OK)
    {
      //size of the chunk data
      std::uint32_t chunk_size =
      ifm3d::mkval<uint32_t >(this->bytes_.data() + invintridx + 4);
      invintridx += pixel_data_offset;
      if (invintrfmt !=
          static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F))
        {
          LOG(ERROR) << "Inverse intrinsic are expected to be float, not: "
                     << invintrfmt;
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
      if (header_version < 2 && (chunk_size - pixel_data_offset)
          != ifm3d::NUM_INTRINSIC_PARAM * sizeof(uint32_t))
        {
          LOG(ERROR) << "Header Version expected value is >=2, not :"
                     << header_version
                     << "Intrinsic param dataLength expected value 64, not :"
                     << chunk_size - pixel_data_offset;

        throw ifm3d::error_t(IFM3D_HEADER_VERSION_MISMATCH);
        }
      for (std::size_t i = 0; i < ifm3d::NUM_INTRINSIC_PARAM; ++i, invintridx += 4)
        {
          this->inverseIntrinsics_[i] =
                ifm3d::mkval<float>(this->bytes_.data()+invintridx);
        }
      this->inverse_intrinsic_available = true;
    }

  if (JSON_OK)
  {
    std::uint32_t chunk_size =
      ifm3d::mkval<uint32_t >(this->bytes_.data() + jsonidx + 4);
    jsonidx += pixel_data_offset; // this is actually header size
    this->json_model_.resize(chunk_size - pixel_data_offset);
    std::memcpy((void*)this->json_model_.data(), (void*)(this->bytes_.data() + jsonidx), chunk_size - pixel_data_offset);
  }

  //
  // extrinsic calibration
  //
  if (EXT_OK)
    {
      //size of the chunk data
      std::uint32_t chunk_size =
      ifm3d::mkval<uint32_t >(this->bytes_.data() + extidx + 4);
      extidx += pixel_data_offset;
      if (extfmt !=
          static_cast<std::uint32_t>(ifm3d::pixel_format::FORMAT_32F))
        {
          LOG(ERROR) << "Extrinsics are expected to be float32, not: "
                     << extfmt;
          throw ifm3d::error_t(IFM3D_PIXEL_FORMAT_ERROR);
        }
      if (header_version < 2 && (chunk_size - pixel_data_offset)
         != ifm3d::NUM_EXTRINSIC_PARAM * sizeof(uint32_t))
        {
          LOG(ERROR) << "Header Version expected value is >= 2, not :"
                     << header_version
                     << "Extrinsic param dataLength expected value 24, not :"
                     << chunk_size - pixel_data_offset;

          throw ifm3d::error_t(IFM3D_HEADER_VERSION_MISMATCH);
        }
      for (std::size_t i = 0; i < ifm3d::NUM_EXTRINSIC_PARAM; ++i, extidx += 4)
        {
          this->extrinsics_[i] =
                ifm3d::mkval<float>(this->bytes_.data()+extidx);
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
      if ((bytes_left >= 18) &&
          std::equal(this->bytes_.begin() + extidx,
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
                    this->exposure_times_.end(), 0);
        }

      // Read temp_illu (9 bytes string + 4 bytes float)
      if ((bytes_left >= 13) &&
          std::equal(this->bytes_.begin() + extidx,
                     this->bytes_.begin() + extidx + 8,
                     std::begin("temp_illu")))
        {
          extime_idx += 9;
          bytes_left -= 9;

          this->illu_temp_ =
            ifm3d::mkval<float>(this->bytes_.data() + extime_idx);

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
      VLOG(IFM3D_PROTO_DEBUG)
        << "illu temp and exposure times skipped (can't trust extidx)";
    }

  this->_SetDirty(false);
}

#endif // __IFM3D_FG_DETAIL_BYTE_BUFFER_HPP__
