/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Guards the hand-written FFmpeg ABI mirror in src/ffmpeg_lib.hpp.
 *
 * ifm3d_rtsp deliberately builds without the FFmpeg headers, so the only thing
 * standing between us and a silently misread struct field is that mirror and
 * the layout probe the loader runs against it. These tests confirm the probe
 * succeeded, that it landed on one of the offsets ffmpeg_lib.hpp predicts, and
 * that the sentinels a freshly allocated frame and packet carry read back as
 * expected through the mirror.
 *
 * Cross-checking the mirror against the real FFmpeg declarations needs the
 * headers, and is therefore done in the ffmpeg build repository rather than
 * here.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "../src/ffmpeg_lib.hpp"

#if defined(IFM3D_RTSP_WITH_FFMPEG)

using namespace ifm3d::rtsp;

TEST(FfmpegAbi, LayoutProbeAgreesWithTheMirror)
{
  const FfmpegApi& api = ffmpeg_api();
  if (!api.available)
    {
      GTEST_SKIP() << "no usable FFmpeg installed: " << api.error;
    }

  const std::array<std::size_t, 2> candidates = {
    ff::FRAME_PTS_OFFSET_WITH_KEY_FRAME,
    ff::FRAME_PTS_OFFSET_WITHOUT_KEY_FRAME};

  EXPECT_NE(
    std::find(candidates.begin(), candidates.end(), api.frame_pts_offset),
    candidates.end())
    << "probe measured pts at offset " << api.frame_pts_offset
    << ", which matches neither candidate ("
    << ff::FRAME_PTS_OFFSET_WITH_KEY_FRAME << " / "
    << ff::FRAME_PTS_OFFSET_WITHOUT_KEY_FRAME << ")";
}

TEST(FfmpegAbi, PtsOffsetFollowsTheLibavutilMajor)
{
  const FfmpegApi& api = ffmpeg_api();
  if (!api.available)
    {
      GTEST_SKIP() << "no usable FFmpeg installed: " << api.error;
    }

  /* The layout is selected from the version rather than probed for the
   * sentinel, because probing cannot disambiguate the two candidates on an
   * ILP32 ABI that aligns int64_t to 8: there they sit 8 bytes apart, which
   * is exactly where pkt_dts lives, and av_frame_alloc() sets that to
   * AV_NOPTS_VALUE too. Pin the rule so an ABI break is caught here rather
   * than by the decoder misreading pts. */
  const unsigned major = api.avutil_version() >> 16U;
  const std::size_t expected = major >= ff::KEY_FRAME_REMOVED_IN_AVUTIL_MAJOR ?
                                 ff::FRAME_PTS_OFFSET_WITHOUT_KEY_FRAME :
                                 ff::FRAME_PTS_OFFSET_WITH_KEY_FRAME;

  EXPECT_EQ(api.frame_pts_offset, expected)
    << "libavutil " << major << " should use offset " << expected;
}

TEST(FfmpegAbi, TheWordBelowPtsDiscriminatesAShiftedLayout)
{
  const FfmpegApi& api = ffmpeg_api();
  if (!api.available)
    {
      GTEST_SKIP() << "no usable FFmpeg installed: " << api.error;
    }

  /* The load-time check reads the word below pts as well as pts itself,
   * because pkt_dts sits one int64 above pts and carries the same sentinel:
   * reading pts alone would also accept an offset 8 bytes too high. This is
   * what makes that second read discriminating -- sample_aspect_ratio (plus
   * any padding) is {0, 1} on a fresh frame, never AV_NOPTS_VALUE. */
  ff::AVFrame* frame = api.av_frame_alloc();
  ASSERT_NE(frame, nullptr);

  const auto read_i64 = [frame](std::size_t offset) {
    std::int64_t value = 0;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    const auto* base = reinterpret_cast<const unsigned char*>(frame);
    std::memcpy(&value, base + offset, sizeof(value));
    return value;
  };

  EXPECT_EQ(read_i64(api.frame_pts_offset), ff::NOPTS_VALUE);
  EXPECT_EQ(read_i64(api.frame_pts_offset + sizeof(std::int64_t)),
            ff::NOPTS_VALUE)
    << "pkt_dts is expected to carry the sentinel too; that is why pts alone "
       "cannot confirm the offset";
  EXPECT_NE(read_i64(api.frame_pts_offset - sizeof(std::int64_t)),
            ff::NOPTS_VALUE)
    << "the word below pts must never read as the sentinel, or the check "
       "cannot rule out an offset that is 8 bytes too high";

  api.av_frame_free(&frame);
}

TEST(FfmpegAbi, SentinelsReadBackThroughTheMirror)
{
  const FfmpegApi& api = ffmpeg_api();
  if (!api.available)
    {
      GTEST_SKIP() << "no usable FFmpeg installed: " << api.error;
    }

  /* The library itself writes these sentinels, so reading them back through
   * the mirror ties our idea of the layout to its own. Reading pts through the
   * measured offset in particular is what the decoder relies on to pair a
   * decoded frame with the access unit it came from. */
  ff::AVFrame* frame = api.av_frame_alloc();
  ASSERT_NE(frame, nullptr);
  EXPECT_EQ(api.frame_pts(frame), ff::NOPTS_VALUE);
  api.av_frame_free(&frame);
  EXPECT_EQ(frame, nullptr);

  ff::AVPacket* packet = api.av_packet_alloc();
  ASSERT_NE(packet, nullptr);
  EXPECT_EQ(FfmpegApi::packet(packet)->pts, ff::NOPTS_VALUE);
  EXPECT_EQ(FfmpegApi::packet(packet)->data, nullptr);
  EXPECT_EQ(FfmpegApi::packet(packet)->size, 0);
  api.av_packet_free(&packet);
  EXPECT_EQ(packet, nullptr);
}

#endif // IFM3D_RTSP_WITH_FFMPEG
