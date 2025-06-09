// -*- c++ -*-
/*
 * Copyright 2022-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IFM3D_FG_FRAME_IMPL_H
#define IFM3D_FG_FRAME_IMPL_H

#include <fmt/format.h>
#include <ifm3d/device/err.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/fg/module_frame_grabber.h>
#include <map>
#include <utility>

namespace ifm3d
{

  // Helper function to map metadata 'type' string to its buffer_id enum
  buffer_id
  MapMetadataToBufferID(const Buffer& buffer)
  {
    const auto& metadata = buffer.metadata();

    const auto& result_node = metadata.at("result");
    const auto& type_node = result_node.at("type");
    if (!type_node.is_string())
      {
        throw ifm3d::Error(
          IFM3D_JSON_ERROR,
          "The 'type' field in buffer metadata is not a string.");
      }

    std::string type_str = type_node.get<std::string>();
    // Look up the string in the LOGICAL_TO_TRANSPORT_MAPPING
    for (const auto& pair : LOGICAL_TO_TRANSPORT_MAPPING)
      {
        if (pair.second.metadata_type_string == type_str)
          {
            return pair.first; // Return the logical buffer_id
          }
      }

    // If the type string is not found in our known mappings
    throw ifm3d::Error(
      IFM3D_BUFFER_ID_NOT_AVAILABLE,
      fmt::format("Unknown buffer type string in metadata: '{}'", type_str));
  }
  //============================================================
  // Impl interface
  //============================================================
  class IFM3D_NO_EXPORT Frame::Impl
  {
  public:
    Impl(BufferDataListMap images,
         std::vector<TimePointT> timestamps,
         uint64_t frame_count);

    std::vector<ifm3d::TimePointT> TimeStamps();

    bool HasBuffer(buffer_id id);

    Buffer& GetBuffer(buffer_id id, std::optional<size_t> index);
    size_t GetBufferCount(buffer_id id);

    std::vector<ifm3d::buffer_id> GetBuffers();

    [[nodiscard]] uint64_t FrameCount() const;

    // NOLINTBEGIN(readability-identifier-naming)
    decltype(std::declval<std::map<buffer_id, BufferList>>().begin())
    begin() noexcept;
    [[nodiscard]] decltype(std::declval<
                             const std::map<buffer_id, BufferList>>()
                             .begin())
    begin() const noexcept;
    decltype(std::declval<std::map<buffer_id, BufferList>>().end())
    end() noexcept;
    [[nodiscard]] decltype(std::declval<
                             const std::map<buffer_id, BufferList>>()
                             .end())
    end() const noexcept;
    // NOLINTEND(readability-identifier-naming)
  protected:
    //---------------------
    // State
    //---------------------
    BufferDataListMap _images;
    std::vector<TimePointT> _timestamps;
    uint32_t _frame_count;

  }; // end: class FrameGrabber::Impl

} // end: namespace ifm3d

inline ifm3d::Frame::Impl::Impl(BufferDataListMap images,
                                std::vector<TimePointT> timestamps,
                                uint64_t frame_count)
  : _images(std::move(images)),
    _timestamps(std::move(timestamps)),
    _frame_count(frame_count)
{}

inline std::vector<ifm3d::TimePointT>
ifm3d::Frame::Impl::TimeStamps()
{
  return _timestamps;
}

inline bool
ifm3d::Frame::Impl::HasBuffer(buffer_id id)
{
  return _images.find(id) != _images.end();
}

inline ifm3d::Buffer&
ifm3d::Frame::Impl::GetBuffer(buffer_id id, std::optional<size_t> index)
{
  // 1. Check if the requested 'id' is a logical ID that's wrapped in a generic
  // transport buffer.
  auto const& logical_transport_it =
    ifm3d::LOGICAL_TO_TRANSPORT_MAPPING.find(id);

  if (logical_transport_it != ifm3d::LOGICAL_TO_TRANSPORT_MAPPING.end())
    {
      // User asked for a specific logical ID (e.g., O3R_ODS_RENDERED_ZONES)
      const auto& transport_info = logical_transport_it->second;
      buffer_id transport_id =
        transport_info.transport_id; // e.g., O3R_RESULT_ARRAY2D
      // Check if we even received the expected transport type
      if (!HasBuffer(transport_id))
        {
          throw ifm3d::Error(
            IFM3D_BUFFER_ID_NOT_AVAILABLE,
            fmt::format("Requested logical buffer_id: {} (transported as {}) "
                        "is not available. "
                        "The generic transport buffer was not received.",
                        std::to_string(static_cast<int>(id)),
                        std::to_string(static_cast<int>(transport_id))));
        }
      // Get the list of buffers for the transport type (e.g., all
      // O3R_RESULT_ARRAY2D buffers)
      auto& transport_buffer_list = images_.at(transport_id);

      // Collect all buffers that match the requested logical ID by metadata
      std::vector<std::reference_wrapper<ifm3d::Buffer>> matching_buffers;
      for (auto& buffer : transport_buffer_list)
        {
          try
            {
              // Use helper function to get the actual specific type ID from
              // this buffer's metadata
              buffer_id actual_specific_id =
                ifm3d::MapMetadataToBufferID(buffer);

              // If the actual specific ID from metadata matches the ID the
              // user requested
              if (actual_specific_id == id)
                {
                  matching_buffers.push_back(std::ref(buffer));
                }
            }
          catch (const ifm3d::Error& e)
            {
              // Log the error for this specific buffer, but continue searching
              // others. This prevents a single malformed buffer from stopping
              // the search. throw ifm3d::Error(fmt::format("Error mapping
              // metadata for a generic buffer ({}): {}",
              //                std::to_string(static_cast<int>(transport_id)),
              //                e.what()));
            }
        }

      // After iterating all possible transport buffers, check if we found any
      // matches
      if (matching_buffers.empty())
        {
          throw ifm3d::Error(
            IFM3D_BUFFER_ID_NOT_AVAILABLE,
            fmt::format("Requested logical buffer_id: {} (transported as {}) "
                        "was received, "
                        "but no matching metadata found within its instances.",
                        std::to_string(static_cast<int>(id)),
                        std::to_string(static_cast<int>(transport_id))));
        }

      // Apply the index to the list of matching buffers
      auto index_value = index.value_or(0);
      if (index_value < matching_buffers.size())
        {
          return matching_buffers[index_value]
            .get(); // Return the found buffer
        }
      else
        {
          throw ifm3d::Error(
            IFM3D_INDEX_OUT_OF_RANGE,
            fmt::format(
              "Requested logical buffer_id: {} (transported as {}), "
              "index = {} out of range. Found only {} matching instances.",
              std::to_string(static_cast<int>(id)),
              std::to_string(static_cast<int>(transport_id)),
              index_value,
              matching_buffers.size()));
        }
    }
  // 2. Original logic for direct buffer_id lookups (when 'id' is not a logical
  // ID that needs special transport handling)
  //    This handles traditional buffers like CONFIDENCE_IMAGE or DIAGNOSTIC
  //    and also if a user explicitly asks for O3R_RESULT_ARRAY2D or
  //    O3R_RESULT_JSON and expects the generic type without specific metadata
  //    filtering.
  else if (HasBuffer(id))
    {
      auto buffer_list = _images.at(id);
      auto index_value = index.value_or(0);
      if (index_value < buffer_list.size())
        {
          return _images.at(id)[index_value];
        }

      throw ifm3d::Error(IFM3D_INDEX_OUT_OF_RANGE,
                         fmt::format("buffer_id: {}, index = {}",
                                     std::to_string(static_cast<int>(id)),
                                     index_value));
    }
  throw ifm3d::Error(
    IFM3D_BUFFER_ID_NOT_AVAILABLE,
    fmt::format("buffer_id: {}", std::to_string(static_cast<int>(id))));
}

inline size_t
ifm3d::Frame::Impl::GetBufferCount(ifm3d::buffer_id id)
{
  return _images.at(id).size();
}

inline std::vector<ifm3d::buffer_id>
ifm3d::Frame::Impl::GetBuffers()
{
  std::vector<buffer_id> keys;

  std::transform(_images.begin(),
                 _images.end(),
                 std::back_inserter(keys),
                 [](const auto& pair) { return pair.first; });

  return keys;
}

inline uint64_t
ifm3d::Frame::Impl::FrameCount() const
{
  return _frame_count;
}

inline decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
                  .begin())
ifm3d::Frame::Impl::begin() noexcept
{
  return _images.begin();
}

inline decltype(std::declval<
                  const std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
                  .begin())
ifm3d::Frame::Impl::begin() const noexcept
{
  return _images.begin();
}

inline decltype(std::declval<std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
                  .end())
ifm3d::Frame::Impl::end() noexcept
{
  return _images.end();
}

inline decltype(std::declval<
                  const std::map<ifm3d::buffer_id, ifm3d::BufferList>>()
                  .end())
ifm3d::Frame::Impl::end() const noexcept
{
  return _images.end();
}

//============================================================
// Impl -- Implementation Details
//============================================================

#endif // IFM3D_FG_FRAME_IMPL_H
