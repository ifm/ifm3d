/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "null_decoder.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <ifm3d/rtsp/video_decoder.h>

namespace ifm3d::rtsp
{
  namespace
  {
    /* Discards every packet and never produces a frame (NAL-only mode). */
    class NullDecoder : public VideoDecoder
    {
    public:
      int
      SendPacket(const std::uint8_t* /*data*/,
                 int /*size*/,
                 std::uint64_t /*pts*/) override
      {
        return 0;
      }

      int
      ReceiveFrame(VideoFrame& /*out*/) override
      {
        /* Never produces a frame; always reports "need more input". */
        return 0;
      }
    };

    class NullDecoderFactory : public VideoDecoderFactory
    {
    public:
      [[nodiscard]] std::string
      Name() const override
      {
        return NULL_DECODER_NAME;
      }

      [[nodiscard]] bool
      IsAvailable() const override
      {
        return true;
      }

      [[nodiscard]] bool
      SupportsCodec(VideoCodec codec) const override
      {
        return codec == VIDEO_CODEC_H264;
      }

      std::unique_ptr<VideoDecoder>
      CreateDecoder(VideoCodec codec) override
      {
        if (codec != VIDEO_CODEC_H264)
          {
            return nullptr;
          }
        return std::make_unique<NullDecoder>();
      }
    };
  } // namespace

  std::unique_ptr<VideoDecoderFactory>
  make_null_decoder_factory()
  {
    return std::make_unique<NullDecoderFactory>();
  }

} // namespace ifm3d::rtsp
