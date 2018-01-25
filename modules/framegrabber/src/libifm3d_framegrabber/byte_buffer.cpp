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

#include <ifm3d/fg/byte_buffer.h>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
#include <string>
#include <glog/logging.h>
#include <ifm3d/camera/logging.h>

//-------------------------------------
// Utilities
//-------------------------------------

const std::size_t ifm3d::IMG_TICKET_SZ = 16;
const std::size_t ifm3d::IMG_BUFF_START = 8;

bool
ifm3d::verify_ticket_buffer(const std::vector<std::uint8_t>& buff)
{
  return ((buff.size() == ifm3d::IMG_TICKET_SZ) &&
          (buff.at(4) == 'L') &&
          (buff.at(14) == '\r') &&
          (buff.at(15) == '\n'));
}

bool
ifm3d::verify_image_buffer(const std::vector<std::uint8_t>& buff)
{
  std::size_t buff_sz = buff.size();

  return ((buff_sz > ifm3d::IMG_BUFF_START) &&
          (std::string(buff.begin()+4,
                       buff.begin()+ifm3d::IMG_BUFF_START) == "star") &&
          (std::string(buff.end()-6, buff.end()-2) == "stop") &&
          (buff.at(buff_sz - 2) == '\r') &&
          (buff.at(buff_sz - 1) == '\n'));
}

std::size_t
ifm3d::get_image_buffer_size(const std::vector<std::uint8_t>& buff)
{
  return std::stoi(std::string(buff.begin()+5, buff.end()));
}

std::size_t
ifm3d::get_chunk_index(const std::vector<std::uint8_t>& buff,
                       ifm3d::image_chunk chunk_type,
                       std::size_t start_idx)
{
  std::size_t idx = start_idx; // start of first chunk
  std::size_t size = buff.size()-6;

  while (idx < size)
    {
      if (static_cast<std::uint32_t>(chunk_type) ==
          ifm3d::mkval<std::uint32_t>(buff.data()+idx))
        {
          return idx;
        }

      // move to the beginning of the next chunk
      std::uint32_t incr = ifm3d::mkval<std::uint32_t>(buff.data()+idx+4);
      if (incr <= 0)
        {
          LOG(WARNING) << "Next chunk is supposedly "
                       << incr << " bytes from the current one ... failing!";
          break;
        }
      idx += incr;
    }

  return std::numeric_limits<std::size_t>::max();
}

//-------------------------------------
// The ByteBuffer class
//-------------------------------------

ifm3d::ByteBuffer::ByteBuffer()
  : dirty_(false)
{ }

ifm3d::ByteBuffer::~ByteBuffer() = default;

ifm3d::ByteBuffer::ByteBuffer(const ifm3d::ByteBuffer& src_buff)
  : ifm3d::ByteBuffer()
{
  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
}

ifm3d::ByteBuffer&
ifm3d::ByteBuffer::operator= (const ifm3d::ByteBuffer& src_buff)
{
  if (this == &src_buff)
    {
      return *this;
    }

  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
  return *this;
}

void
ifm3d::ByteBuffer::SetBytes(std::vector<std::uint8_t>& buff,
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

void
ifm3d::ByteBuffer::_SetDirty(bool flg) noexcept
{
  this->dirty_ = flg;
}

bool
ifm3d::ByteBuffer::Dirty() const noexcept
{
  return this->dirty_;
}

std::vector<std::uint8_t>
ifm3d::ByteBuffer::Bytes()
{
  return this->bytes_;
}

void
ifm3d::ByteBuffer::Organize()
{
  // This is a basic stub for subclasses
  if (! this->Dirty())
    {
      return;
    }

  // Theoretically, here is where you would iterate over the
  // internally wrapped `bytes_` and populate your image data
  // structures. Always flag as "not dirty" when done.

  this->_SetDirty(false);
}
