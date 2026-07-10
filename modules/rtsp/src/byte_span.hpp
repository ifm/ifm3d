/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_RTSP_BYTE_SPAN_HPP
#define IFM3D_RTSP_BYTE_SPAN_HPP

/** @file
 * @brief Minimal non-owning view over a contiguous range of bytes.
 *
 * ifm3d targets C++17, which predates std::span. This lightweight stand-in
 * provides just the read-only surface the RTSP transport layer needs.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ifm3d::rtsp
{
  /** Read-only, non-owning view over a contiguous byte range. */
  class ByteSpan
  {
  public:
    ByteSpan() = default;
    ByteSpan(const std::uint8_t* data, std::size_t size)
      : _data(data),
        _size(size)
    {}
    ByteSpan(const std::vector<std::uint8_t>& v) // NOLINT(*-explicit-*)
      : _data(v.data()),
        _size(v.size())
    {}

    [[nodiscard]] const std::uint8_t*
    Data() const
    {
      return _data;
    }
    [[nodiscard]] std::size_t
    Size() const
    {
      return _size;
    }
    [[nodiscard]] bool
    Empty() const
    {
      return _size == 0;
    }

    [[nodiscard]] const std::uint8_t*
    Begin() const
    {
      return _data;
    }
    [[nodiscard]] const std::uint8_t*
    End() const
    {
      return _data + _size;
    }

    const std::uint8_t&
    operator[](std::size_t i) const
    {
      return _data[i];
    }

  private:
    const std::uint8_t* _data = nullptr;
    std::size_t _size = 0;
  };

} // namespace ifm3d::rtsp

#endif // IFM3D_RTSP_BYTE_SPAN_HPP
