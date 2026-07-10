/**
 * Copyright (C) 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ifm3d/rtsp/decoder_manager.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <ifm3d/common/logging/log.h>
#include <ifm3d/rtsp/video_decoder.h>

#include "null_decoder.hpp"

#ifdef IFM3D_RTSP_WITH_FFMPEG
#  include "ffmpeg_decoder.hpp"
#endif

namespace
{
  using ifm3d::rtsp::VideoDecoderFactory;

  /* Build the registry of compiled-in decoder implementations, in preference
   * order: real decoders first, the always-available null decoder last. */
  std::vector<std::unique_ptr<VideoDecoderFactory>>
  build_registry()
  {
    std::vector<std::unique_ptr<VideoDecoderFactory>> factories;
#ifdef IFM3D_RTSP_WITH_FFMPEG
    factories.push_back(ifm3d::rtsp::make_ffmpeg_decoder_factory());
#endif
    factories.push_back(ifm3d::rtsp::make_null_decoder_factory());
    return factories;
  }

} // namespace

// ---------------------------------------------------------------------------
// ifm3d::DecoderManager
// ---------------------------------------------------------------------------

namespace ifm3d
{

  DecoderManager::DecoderManager(std::optional<std::string> preferred_decoder)
    : _preferred_decoder(std::move(preferred_decoder)),
      _factories(build_registry())
  {}

  DecoderManager::~DecoderManager() = default;

  std::vector<DecoderManager::DecoderInfo>
  DecoderManager::DiscoverDecoders()
  {
    std::vector<DecoderInfo> result;
    for (const auto& factory : build_registry())
      {
        DecoderInfo info;
        info.name = factory->Name();
        info.available = factory->IsAvailable();
        info.supports_h264 =
          factory->SupportsCodec(ifm3d::rtsp::VIDEO_CODEC_H264);
        if (!info.available)
          {
            info.error = factory->AvailabilityError();
          }
        result.push_back(std::move(info));
      }
    return result;
  }

  void
  DecoderManager::LoadDecoders()
  {
    select_decoders();
  }

  void
  DecoderManager::select_decoders()
  {
    const ifm3d::rtsp::VideoCodec codec = ifm3d::rtsp::VIDEO_CODEC_H264;

    VideoDecoderFactory* null_factory = nullptr;
    for (const auto& factory : _factories)
      {
        if (factory->Name() == ifm3d::rtsp::NULL_DECODER_NAME)
          {
            null_factory = factory.get();
            break;
          }
      }

    VideoDecoderFactory* chosen = nullptr;
    bool explicit_null = false;

    if (_preferred_decoder &&
        *_preferred_decoder == ifm3d::rtsp::NULL_DECODER_NAME)
      {
        /* Null decoder requested explicitly. */
        chosen = null_factory;
        explicit_null = true;
      }
    else if (_preferred_decoder && !_preferred_decoder->empty())
      {
        /* Name-based selection: first available real decoder whose name
         * contains the requested string and supports the codec. */
        for (const auto& factory : _factories)
          {
            if (factory->Name() == ifm3d::rtsp::NULL_DECODER_NAME)
              {
                continue;
              }
            if (factory->Name().find(*_preferred_decoder) !=
                  std::string::npos &&
                factory->IsAvailable() && factory->SupportsCodec(codec))
              {
                chosen = factory.get();
                break;
              }
          }
        if (chosen == nullptr)
          {
            LOG_WARNING(
              "[DecoderManager] Requested decoder '{}' not available; "
              "falling back to automatic selection",
              *_preferred_decoder);
          }
      }

    if (chosen == nullptr)
      {
        /* Automatic selection: first available real decoder for the codec. */
        for (const auto& factory : _factories)
          {
            if (factory->Name() == ifm3d::rtsp::NULL_DECODER_NAME)
              {
                continue;
              }
            if (factory->IsAvailable() && factory->SupportsCodec(codec))
              {
                chosen = factory.get();
                break;
              }
          }
      }

    if (chosen == nullptr)
      {
        /* No real decoder available: fall back to the null decoder. */
        chosen = null_factory;
      }

    if (chosen == null_factory)
      {
        if (explicit_null)
          {
            LOG_WARNING(
              "[DecoderManager] Using the 'null' decoder as requested; "
              "decoded frames will not be produced (NAL-only)");
          }
        else
          {
            LOG_WARNING(
              "[DecoderManager] No H264 decoder available; using the "
              "'null' decoder (NAL-only). Decoded frames will not be "
              "produced");
          }
      }
    else
      {
        LOG_INFO("[DecoderManager] Selected decoder: {}", chosen->Name());
      }

    if (chosen != nullptr)
      {
        _selected[codec] = chosen;
      }
  }

  std::unique_ptr<ifm3d::rtsp::VideoDecoder>
  DecoderManager::CreateDecoder(ifm3d::rtsp::VideoCodec codec)
  {
    auto it = _selected.find(codec);
    if (it == _selected.end() || it->second == nullptr)
      {
        return nullptr;
      }
    return it->second->CreateDecoder(codec);
  }

  std::string
  DecoderManager::SelectedDecoderName(ifm3d::rtsp::VideoCodec codec) const
  {
    auto it = _selected.find(codec);
    if (it == _selected.end() || it->second == nullptr)
      {
        return {};
      }
    return it->second->Name();
  }

} // namespace ifm3d
