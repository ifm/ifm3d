/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_BIT_READER_WRITER_HPP
#define IFM3D_RTSP_BIT_READER_WRITER_HPP

/** @file
 * @brief BitReader / BitWriter for destructuring/constructing RTP/RTCP
 * headers and big-endian helpers.
 *
 * Reads/writes bits from network data in MSB-first (big-endian) order.
 * Ported from the o3c-rtsp-client reference and kept header-only for use by
 * the RTP demultiplexer and the H.264 depacketizer.
 */

#include <bitset>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "byte_span.hpp"

namespace ifm3d::rtsp
{
  /**
   * @brief Compile-time descriptor of a bit field at a given bit offset.
   *
   * @tparam Start First bit (MSB-first) of the field.
   * @tparam Size  Number of bits in the field.
   */
  template <uint8_t START_BIT,
            uint8_t NUM_BITS,
            typename T = std::conditional_t<
              NUM_BITS == 1,
              bool,
              std::conditional_t<
                NUM_BITS <= 8,
                uint8_t,
                std::conditional_t<
                  NUM_BITS <= 16,
                  uint16_t,
                  std::conditional_t<NUM_BITS <= 32, uint32_t, uint64_t>>>>>
  struct BitField
  {
    static constexpr uint8_t START = START_BIT;
    static constexpr uint8_t SIZE = NUM_BITS;
    using TYPE = T;
    static_assert(sizeof(T) * 8 >= NUM_BITS,
                  "BitField result type is too small for Size.");
  };

  namespace detail
  {
    template <typename T>
    T
    from_be(T v)
    {
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
      return v;
#else
      if constexpr (sizeof(T) == 1)
        {
          return v;
        }
      else if constexpr (sizeof(T) == 2)
        {
          return static_cast<T>(__builtin_bswap16(static_cast<uint16_t>(v)));
        }
      else if constexpr (sizeof(T) == 4)
        {
          return static_cast<T>(__builtin_bswap32(static_cast<uint32_t>(v)));
        }
      else
        {
          return static_cast<T>(__builtin_bswap64(static_cast<uint64_t>(v)));
        }
#endif
    }
  } // namespace detail

  /**
   * @brief Reads bit fields from a raw byte buffer in MSB-first order.
   *
   * The reader does not own the buffer; the caller must keep it alive.
   */
  class BitReader
  {
  public:
    explicit BitReader(const uint8_t* data) : _data(data) {}
    explicit BitReader(const std::vector<uint8_t>& data) : _data(data.data())
    {}
    explicit BitReader(ByteSpan data) : _data(data.Data()) {}

    template <typename T, uint8_t START, uint8_t SIZE>
    T
    Read() const
    {
      std::bitset<SIZE> result;
      for (int i = 0; i < SIZE; ++i)
        {
          result[SIZE - 1 - i] = bit(START + i);
        }
      return static_cast<T>(result.to_ullong());
    }

    template <typename BF_TYPE>
    typename BF_TYPE::TYPE
    Read() const
    {
      return Read<typename BF_TYPE::TYPE, BF_TYPE::START, BF_TYPE::SIZE>();
    }

  private:
    [[nodiscard]] bool
    bit(size_t bit) const
    {
      return ((_data[bit / 8] >> (7 - (bit % 8))) & 0x01) != 0;
    }

    const uint8_t* _data;
  };

  /**
   * @brief Writes bit fields into a raw byte buffer in MSB-first order.
   */
  class BitWriter
  {
  public:
    explicit BitWriter(uint8_t* data) : _data(data) {}
    explicit BitWriter(std::vector<uint8_t>& data) : _data(data.data()) {}

    template <typename T, uint8_t START, uint8_t SIZE>
    void
    Write(T val)
    {
      std::bitset<SIZE> bits(static_cast<unsigned long long>(val));
      for (int i = 0; i < SIZE; ++i)
        {
          set_bit(START + i, bits[SIZE - 1 - i]);
        }
    }

    template <typename BF_TYPE>
    void
    Write(typename BF_TYPE::TYPE val)
    {
      Write<typename BF_TYPE::TYPE, BF_TYPE::START, BF_TYPE::SIZE>(val);
    }

  private:
    void
    set_bit(size_t bit, bool val)
    {
      const size_t idx = bit / 8;
      const size_t shift = 7 - (bit % 8);
      _data[idx] = static_cast<uint8_t>((_data[idx] & ~(0x01U << shift)) |
                                        (static_cast<uint8_t>(val) << shift));
    }

    uint8_t* _data;
  };

  /** @brief Convenience read of a big-endian uint16 at a byte offset. */
  namespace big_endian
  {
    inline uint16_t
    read_u_int16(const std::vector<uint8_t>& buf, size_t offset)
    {
      return static_cast<uint16_t>((buf[offset] << 8) | buf[offset + 1]);
    }

    inline uint16_t
    read_u_int16(const uint8_t* data)
    {
      return static_cast<uint16_t>((data[0] << 8) | data[1]);
    }
  } // namespace big_endian

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_BIT_READER_WRITER_HPP
