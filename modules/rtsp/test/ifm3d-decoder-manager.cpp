/*
 * Copyright (C) 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for ifm3d::DecoderManager.
 *
 * Decoders are compiled into the RTSP module; the ffmpeg decoder resolves
 * libavcodec at runtime, so tests that require a real H.264 decoder are
 * guarded on its runtime availability.
 */

#include <gtest/gtest.h>

#include <ifm3d/rtsp/decoder_manager.h>
#include <ifm3d/rtsp/video_decoder.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using ifm3d::DecoderManager;
using ifm3d::rtsp::VIDEO_CODEC_H264;
using ifm3d::rtsp::VideoCodec;

namespace
{
  // Whether the ffmpeg decoder is compiled in AND usable on this system
  // (i.e. a compatible libavcodec could be loaded at runtime).
  bool
  ffmpeg_available()
  {
    for (const auto& info : DecoderManager::DiscoverDecoders())
      {
        if (info.name == "ffmpeg")
          {
            return info.available;
          }
      }
    return false;
  }
} // namespace

// ---------------------------------------------------------------------------
// DiscoverDecoders (registry listing)
// ---------------------------------------------------------------------------

TEST(DecoderManager, DiscoverDecodersAlwaysListsNull)
{
  const auto decoders = DecoderManager::DiscoverDecoders();
  ASSERT_FALSE(decoders.empty());

  const auto it = std::find_if(decoders.begin(),
                               decoders.end(),
                               [](const auto& p) { return p.name == "null"; });
  ASSERT_NE(it, decoders.end()) << "the null decoder must always be listed";
  EXPECT_TRUE(it->available) << "the null decoder is always available";
  EXPECT_TRUE(it->supports_h264);
  EXPECT_TRUE(it->error.empty());
}

TEST(DecoderManager, DiscoverDecodersIsConsistent)
{
  // available and error must never disagree.
  for (const auto& p : DecoderManager::DiscoverDecoders())
    {
      EXPECT_EQ(p.available, p.error.empty())
        << "inconsistent result for: " << p.name;
    }
}

#ifdef IFM3D_RTSP_WITH_FFMPEG
TEST(DecoderManager, DiscoverDecodersListsFfmpegWhenCompiledIn)
{
  const auto decoders = DecoderManager::DiscoverDecoders();
  const auto it =
    std::find_if(decoders.begin(), decoders.end(), [](const auto& p) {
      return p.name == "ffmpeg";
    });
  ASSERT_NE(it, decoders.end()) << "ffmpeg must be listed when compiled in";
  // available reflects runtime libavcodec availability; when available it must
  // support H.264 and report no error.
  if (it->available)
    {
      EXPECT_TRUE(it->supports_h264);
      EXPECT_TRUE(it->error.empty());
    }
  else
    {
      EXPECT_FALSE(it->error.empty());
    }
}
#endif

// ---------------------------------------------------------------------------
// Selection
// ---------------------------------------------------------------------------

TEST(DecoderManager, NoDecoderBeforeLoadDecoders)
{
  // CreateDecoder must return nullptr before LoadDecoders() runs.
  DecoderManager manager;
  EXPECT_EQ(manager.CreateDecoder(VIDEO_CODEC_H264), nullptr);
  EXPECT_TRUE(manager.SelectedDecoderName(VIDEO_CODEC_H264).empty());
}

TEST(DecoderManager, ADecoderIsAlwaysAvailableAfterLoad)
{
  // Even with no real decoder, the null fallback yields a decoder.
  DecoderManager manager;
  manager.LoadDecoders();
  auto dec = manager.CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);
  EXPECT_FALSE(manager.SelectedDecoderName(VIDEO_CODEC_H264).empty());
}

TEST(DecoderManager, ExplicitNullIsSelected)
{
  DecoderManager manager(/*preferred_decoder=*/"null");
  manager.LoadDecoders();
  EXPECT_EQ(manager.SelectedDecoderName(VIDEO_CODEC_H264), "null");
  auto dec = manager.CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);
}

TEST(DecoderManager, UnknownPreferredFallsBack)
{
  DecoderManager manager(/*preferred_decoder=*/"does-not-exist");
  manager.LoadDecoders();
  // Selection still yields some decoder (real if available, else null).
  EXPECT_FALSE(manager.SelectedDecoderName(VIDEO_CODEC_H264).empty());
}

TEST(DecoderManager, LoadDecodersTwiceIsIdempotent)
{
  DecoderManager manager;
  manager.LoadDecoders();
  const std::string first = manager.SelectedDecoderName(VIDEO_CODEC_H264);
  manager.LoadDecoders();
  EXPECT_EQ(manager.SelectedDecoderName(VIDEO_CODEC_H264), first);
  EXPECT_NE(manager.CreateDecoder(VIDEO_CODEC_H264), nullptr);
}

// ---------------------------------------------------------------------------
// Decoder lifecycle
// ---------------------------------------------------------------------------

TEST(DecoderManager, CreateDecoderReturnsDistinctInstances)
{
  DecoderManager manager;
  manager.LoadDecoders();
  auto dec1 = manager.CreateDecoder(VIDEO_CODEC_H264);
  auto dec2 = manager.CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec1, nullptr);
  ASSERT_NE(dec2, nullptr);
  EXPECT_NE(dec1.get(), dec2.get())
    << "each call must return an independent instance";
}

TEST(DecoderManager, UnknownCodecReturnsNullDecoder)
{
  DecoderManager manager;
  manager.LoadDecoders();
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  const auto invalid_codec = static_cast<VideoCodec>(0xFFFF);
  EXPECT_EQ(manager.CreateDecoder(invalid_codec), nullptr);
}

TEST(DecoderManager, NullDecoderDiscardsPackets)
{
  // Force the null decoder and exercise its interface.
  DecoderManager manager(/*preferred_decoder=*/"null");
  manager.LoadDecoders();
  auto dec = manager.CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);

  const std::vector<std::uint8_t> payload = {0x00, 0x00, 0x00, 0x01, 0x67};
  EXPECT_EQ(dec->SendPacket(payload.data(), static_cast<int>(payload.size())),
            0);

  ifm3d::rtsp::VideoFrame frame{};
  EXPECT_EQ(dec->ReceiveFrame(frame), 0)
    << "null decoder never yields a frame";
}

#ifdef IFM3D_RTSP_WITH_FFMPEG
TEST(DecoderManager, FfmpegSelectedWhenAvailable)
{
  if (!ffmpeg_available())
    {
      GTEST_SKIP() << "libavcodec not available at runtime";
    }
  DecoderManager manager(/*preferred_decoder=*/"ffmpeg");
  manager.LoadDecoders();
  EXPECT_EQ(manager.SelectedDecoderName(VIDEO_CODEC_H264), "ffmpeg");
  auto dec = manager.CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);
}

TEST(DecoderManager, FfmpegIsDefaultWhenAvailable)
{
  if (!ffmpeg_available())
    {
      GTEST_SKIP() << "libavcodec not available at runtime";
    }
  DecoderManager manager;
  manager.LoadDecoders();
  EXPECT_EQ(manager.SelectedDecoderName(VIDEO_CODEC_H264), "ffmpeg")
    << "ffmpeg should be preferred over null when available";
}
#endif
