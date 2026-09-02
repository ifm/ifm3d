/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the RTSP client building blocks (no network required):
 * SDP parsing, the H.264 depacketizer, SEI / RGB_INFO parsing, the bit
 * reader and the decoder host's NAL-only fallback.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include <asio/buffer.hpp>
#include <asio/error_code.hpp>
#include <asio/executor_work_guard.hpp>
#include <asio/io_context.hpp>
#include <asio/ip/address_v4.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/post.hpp>
#include <asio/steady_timer.hpp>

#include <ifm3d/common/err.h>
#include <ifm3d/deserialize/deserialize.h>
#include <ifm3d/deserialize/struct_rgb_info_v1.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/rtsp/nal_unit.h>

#include "bit_reader_writer.hpp"
#include "byte_span.hpp"
#include "decoder_host.hpp"
#include "h264_depacketizer.hpp"
#include "package_decoder.hpp"
#include "rtp_client.hpp"
#include "rtp_connection.hpp"
#include "rtp_connection_udp.hpp"
#include "rtsp_session.hpp"
#include "rtsp_util.hpp"
#include "sdp_parser.hpp"
#include "sei_parser.hpp"

using namespace ifm3d::rtsp;

// ---------------------------------------------------------------------------
// SDP parser
// ---------------------------------------------------------------------------

TEST(SdpParser, ParsesH264TrackAndSprop)
{
  const std::string sdp =
    "v=0\r\n"
    "o=- 0 0 IN IP4 127.0.0.1\r\n"
    "s=Session\r\n"
    "m=video 0 RTP/AVP 96\r\n"
    "a=rtpmap:96 H264/90000\r\n"
    "a=fmtp:96 packetization-mode=1;sprop-parameter-sets=Z0LAH,aM4G\r\n"
    "a=control:trackID=0\r\n";

  const SdpInfo info = parse_sdp(sdp, "rtsp://192.168.0.69:8554/port1");

  EXPECT_TRUE(info.has_h264);
  EXPECT_EQ(info.payload_type, 96);
  EXPECT_EQ(info.sprop_parameter_sets, "Z0LAH,aM4G");
  EXPECT_EQ(info.track_url, "rtsp://192.168.0.69:8554/port1/trackID=0");
}

TEST(SdpParser, AbsoluteControlUrlIsKept)
{
  const std::string sdp = "m=video 0 RTP/AVP 98\r\n"
                          "a=rtpmap:98 H264/90000\r\n"
                          "a=control:rtsp://host/abs/track\r\n";

  const SdpInfo info = parse_sdp(sdp, "rtsp://host/base");
  EXPECT_EQ(info.payload_type, 98);
  EXPECT_EQ(info.track_url, "rtsp://host/abs/track");
}

TEST(SdpParser, NoH264TrackReportsAbsence)
{
  const std::string sdp = "m=audio 0 RTP/AVP 0\r\n"
                          "a=rtpmap:0 PCMU/8000\r\n";
  const SdpInfo info = parse_sdp(sdp, "rtsp://host/base");
  EXPECT_FALSE(info.has_h264);
}

// ---------------------------------------------------------------------------
// BitReader
// ---------------------------------------------------------------------------

TEST(BitReader, ReadsRtpHeaderFields)
{
  // version=2, padding=0, ext=0, cc=0, marker=1, pt=96, seq=0x1234
  const std::vector<std::uint8_t> data = {0x80, 0xE0, 0x12, 0x34};
  const BitReader r(data);
  EXPECT_EQ((r.Read<std::uint8_t, 0, 2>()), 2);  // version
  EXPECT_EQ((r.Read<std::uint8_t, 8, 1>()), 1);  // marker
  EXPECT_EQ((r.Read<std::uint8_t, 9, 7>()), 96); // payload type
  EXPECT_EQ((r.Read<std::uint16_t, 16, 16>()), 0x1234);
}

// ---------------------------------------------------------------------------
// H264 depacketizer
// ---------------------------------------------------------------------------

TEST(H264Depacketizer, SingleNalEmitsNalAndAccessUnit)
{
  H264Depacketizer dp;

  std::vector<ifm3d::NalUnit> nals;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_nal_unit = [&](const ifm3d::NalUnit& n) { nals.push_back(n); };
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  // A single non-IDR slice NAL (type 1), nal_ref_idc 2 -> header 0x41.
  const std::vector<std::uint8_t> payload = {0x41, 0xAA, 0xBB, 0xCC};
  dp.DecodePackage(payload, 9000, 1);
  dp.FinishFrame();

  ASSERT_EQ(nals.size(), 1U);
  EXPECT_EQ(nals[0].nal_unit_type,
            static_cast<std::uint8_t>(ifm3d::NalUnit::NON_IDR_SLICE));
  EXPECT_EQ(nals[0].nal_ref_idc, 2);
  EXPECT_FALSE(nals[0].is_idr);
  EXPECT_EQ(nals[0].pts_us, 100000U); // 9000 / 90000 * 1e6

  ASSERT_EQ(access_units.size(), 1U);
  const std::vector<std::uint8_t> expected =
    {0x00, 0x00, 0x00, 0x01, 0x41, 0xAA, 0xBB, 0xCC};
  EXPECT_EQ(access_units[0], expected);
}

TEST(H264Depacketizer, FuAReassemblesFragmentedIdr)
{
  H264Depacketizer dp;

  std::vector<ifm3d::NalUnit> nals;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_nal_unit = [&](const ifm3d::NalUnit& n) { nals.push_back(n); };
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  // Original NAL: IDR slice (type 5), nal_ref_idc 3 -> header byte 0x65.
  // FU indicator: f=0,nri=3,type=28(FU-A) -> 0x7C.
  // FU header start: s=1,e=0,type=5 -> 0x85. End: s=0,e=1,type=5 -> 0x45.
  const std::vector<std::uint8_t> frag1 = {0x7C, 0x85, 0x11, 0x22};
  const std::vector<std::uint8_t> frag2 = {0x7C, 0x45, 0x33, 0x44};
  dp.DecodePackage(frag1, 18000, 10);
  dp.DecodePackage(frag2, 18000, 11);
  dp.FinishFrame();

  ASSERT_EQ(nals.size(), 1U);
  EXPECT_EQ(nals[0].nal_unit_type,
            static_cast<std::uint8_t>(ifm3d::NalUnit::IDR_SLICE));
  EXPECT_TRUE(nals[0].is_idr);
  EXPECT_EQ(nals[0].first_sequence_number, 10);
  EXPECT_EQ(nals[0].last_sequence_number, 11);

  const std::vector<std::uint8_t> expected_nal = {0x65,
                                                  0x11,
                                                  0x22,
                                                  0x33,
                                                  0x44};
  EXPECT_EQ(nals[0].data, expected_nal);

  ASSERT_EQ(access_units.size(), 1U);
}

TEST(H264Depacketizer, AccessUnitContainsAllSlicesInReceivingOrder)
{
  H264Depacketizer dp;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  // first_mb_in_slice = 0 ("1"), followed by first_mb_in_slice = 1 ("010").
  dp.DecodePackage({0x41, 0x80, 0x11}, 9000, 1);
  dp.DecodePackage({0x41, 0x40, 0x22}, 9000, 2);
  dp.FinishFrame();

  ASSERT_EQ(access_units.size(), 1U);
  const std::vector<std::uint8_t> expected = {0x00,
                                              0x00,
                                              0x00,
                                              0x01,
                                              0x41,
                                              0x80,
                                              0x11,
                                              0x00,
                                              0x00,
                                              0x00,
                                              0x01,
                                              0x41,
                                              0x40,
                                              0x22};
  EXPECT_EQ(access_units.front(), expected);
}

TEST(H264Depacketizer, AccessUnitContainsAllNalUnitsInReceivingOrder)
{
  H264Depacketizer dp;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  dp.DecodePackage({0x67, 0x11}, 9000, 1);
  dp.DecodePackage({0x68, 0x22}, 9000, 2);
  dp.DecodePackage({0x06, 0x80}, 9000, 3);
  dp.DecodePackage({0x41, 0x80, 0x33}, 9000, 4);
  dp.DecodePackage({0x41, 0x40, 0x44}, 9000, 5);
  dp.FinishFrame();

  ASSERT_EQ(access_units.size(), 1U);
  const std::vector<std::uint8_t> expected = {
    0x00, 0x00, 0x00, 0x01, 0x67, 0x11, 0x00, 0x00, 0x00, 0x01, 0x68,
    0x22, 0x00, 0x00, 0x00, 0x01, 0x06, 0x80, 0x00, 0x00, 0x00, 0x01,
    0x41, 0x80, 0x33, 0x00, 0x00, 0x00, 0x01, 0x41, 0x40, 0x44};
  EXPECT_EQ(access_units.front(), expected);
}

TEST(H264Depacketizer, FirstSliceStartsNextAccessUnitWithoutAud)
{
  H264Depacketizer dp;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  dp.DecodePackage({0x41, 0x80, 0x11}, 9000, 1);
  dp.DecodePackage({0x41, 0x80, 0x22}, 18000, 2);

  ASSERT_EQ(access_units.size(), 1U);
  const std::vector<std::uint8_t> expected =
    {0x00, 0x00, 0x00, 0x01, 0x41, 0x80, 0x11};
  EXPECT_EQ(access_units.front(), expected);
}

TEST(H264Depacketizer, RtpTimestampStartsNextAccessUnit)
{
  H264Depacketizer dp;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  dp.DecodePackage({0x41, 0x40, 0x11}, 9000, 1);
  dp.DecodePackage({0x41, 0x40, 0x22}, 18000, 2);

  ASSERT_EQ(access_units.size(), 1U);
  const std::vector<std::uint8_t> expected =
    {0x00, 0x00, 0x00, 0x01, 0x41, 0x40, 0x11};
  EXPECT_EQ(access_units.front(), expected);
}

TEST(H264Depacketizer, TruncatedSliceHeaderDoesNotSplitAccessUnit)
{
  H264Depacketizer dp;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  dp.DecodePackage({0x41, 0x40, 0x11}, 9000, 1);
  dp.DecodePackage({0x41}, 9000, 2);

  EXPECT_TRUE(access_units.empty());
  dp.FinishFrame();
  ASSERT_EQ(access_units.size(), 1U);
}

TEST(H264Depacketizer, CancelFrameReportsTheDiscardedAccessUnit)
{
  H264Depacketizer dp;
  int cancelled = 0;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_access_unit_cancelled = [&] { ++cancelled; };
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  // An access unit that is abandoned part-way, as happens after a sequence
  // gap, must tell the client so that anything derived from the NALs it did
  // receive is dropped instead of leaking into the next access unit.
  dp.DecodePackage({0x41, 0x40, 0x11}, 9000, 1);
  dp.CancelFrame();

  EXPECT_EQ(cancelled, 1);
  EXPECT_TRUE(access_units.empty());

  dp.DecodePackage({0x41, 0x40, 0x22}, 18000, 3);
  dp.FinishFrame();

  EXPECT_EQ(cancelled, 1);
  ASSERT_EQ(access_units.size(), 1U);
}

namespace
{
  // SEI NAL (type 6): payloadType=5 (unregistered user data), payloadSize=16
  // (UUID only), the fallback-video marker UUID and the RBSP stop bit.
  std::vector<std::uint8_t>
  make_fallback_video_sei()
  {
    std::vector<std::uint8_t> sei = {0x06, 0x05, 0x10};
    sei.insert(sei.end(),
               FALLBACK_VIDEO_UUID.begin(),
               FALLBACK_VIDEO_UUID.end());
    sei.push_back(0x80);
    return sei;
  }
} // namespace

TEST(H264Depacketizer, FallbackVideoMarkerSeiDropsAccessUnit)
{
  H264Depacketizer dp;
  int fallback = 0;
  int cancelled = 0;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_access_unit_cancelled = [&] { ++cancelled; };
  dp.on_fallback_access_unit = [&] { ++fallback; };
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  dp.DecodePackage(make_fallback_video_sei(), 9000, 1);
  dp.DecodePackage({0x41, 0x80, 0x11}, 9000, 2);
  dp.FinishFrame();

  EXPECT_TRUE(access_units.empty());
  EXPECT_EQ(fallback, 1);
  EXPECT_EQ(cancelled, 0) << "a dropped fallback frame is not a stream gap";

  // A subsequent access unit without the marker is emitted normally.
  dp.DecodePackage({0x41, 0x80, 0x22}, 18000, 3);
  dp.FinishFrame();

  ASSERT_EQ(access_units.size(), 1U);
  EXPECT_EQ(fallback, 1);
}

TEST(H264Depacketizer, FallbackVideoMarkerIsFoundInAStapAAggregate)
{
  H264Depacketizer dp;
  int fallback = 0;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_fallback_access_unit = [&] { ++fallback; };
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  // STAP-A (type 24, nri 3 -> 0x78) aggregating the marker SEI and a slice,
  // each prefixed by its 16-bit size. This is how the device packs the two.
  const std::vector<std::uint8_t> sei = make_fallback_video_sei();
  const std::vector<std::uint8_t> slice = {0x41, 0x80, 0x11};

  std::vector<std::uint8_t> stap = {0x78};
  stap.push_back(0x00);
  stap.push_back(static_cast<std::uint8_t>(sei.size()));
  stap.insert(stap.end(), sei.begin(), sei.end());
  stap.push_back(0x00);
  stap.push_back(static_cast<std::uint8_t>(slice.size()));
  stap.insert(stap.end(), slice.begin(), slice.end());

  dp.DecodePackage(stap, 9000, 1);
  dp.FinishFrame();

  EXPECT_TRUE(access_units.empty());
  EXPECT_EQ(fallback, 1);
}

TEST(H264Depacketizer, FallbackVideoAccessUnitLeavesSpropForTheNextOne)
{
  H264Depacketizer dp;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  // base64 of SPS {0x67,0x42,0x00,0x0A} and PPS {0x68,0xCE,0x3C,0x80}.
  dp.SeedFromSprop("Z0IACg==,aM48gA==");

  dp.DecodePackage(make_fallback_video_sei(), 9000, 1);
  dp.DecodePackage({0x41, 0x80, 0x11}, 9000, 2);
  dp.FinishFrame();

  dp.DecodePackage({0x41, 0x80, 0x22}, 18000, 3);
  dp.FinishFrame();

  // The parameter sets went out with the dropped access unit, so the first
  // access unit the application actually sees must still carry them.
  ASSERT_EQ(access_units.size(), 1U);
  const std::vector<std::uint8_t> expected = {
    0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01,
    0x68, 0xCE, 0x3C, 0x80, 0x00, 0x00, 0x00, 0x01, 0x41, 0x80, 0x22};
  EXPECT_EQ(access_units.front(), expected);
}

TEST(H264Depacketizer, FallbackVideoMarkerWithoutSliceDoesNotDropTheNextFrame)
{
  H264Depacketizer dp;
  int fallback = 0;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_fallback_access_unit = [&] { ++fallback; };
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  // The RTP marker bit ends an access unit whose slices never arrived. The
  // marker described that access unit and must not be carried into the next.
  dp.DecodePackage(make_fallback_video_sei(), 9000, 1);
  dp.FinishFrame();

  EXPECT_TRUE(access_units.empty());
  EXPECT_EQ(fallback, 0) << "no slice arrived, so no frame was dropped";

  dp.DecodePackage({0x41, 0x80, 0x22}, 18000, 2);
  dp.FinishFrame();

  EXPECT_EQ(fallback, 0);
  ASSERT_EQ(access_units.size(), 1U);
}

TEST(H264Depacketizer, ConsecutiveFallbackFramesAreAllDroppedUntilARealOne)
{
  H264Depacketizer dp;
  int fallback = 0;
  int cancelled = 0;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_access_unit_cancelled = [&] { ++cancelled; };
  dp.on_fallback_access_unit = [&] { ++fallback; };
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };

  // base64 of SPS {0x67,0x42,0x00,0x0A} and PPS {0x68,0xCE,0x3C,0x80}.
  dp.SeedFromSprop("Z0IACg==,aM48gA==");

  // The device streams fallback frames back-to-back while no real image is
  // available, so a burst of drops -- not a single one -- is the expected
  // input.
  std::uint32_t rtp_timestamp = 9000;
  std::uint16_t sequence_number = 1;
  for (int i = 0; i < 3; ++i)
    {
      dp.DecodePackage(make_fallback_video_sei(),
                       rtp_timestamp,
                       sequence_number++);
      dp.DecodePackage({0x41, 0x80, 0x11}, rtp_timestamp, sequence_number++);
      dp.FinishFrame();
      rtp_timestamp += 9000;
    }

  dp.DecodePackage({0x41, 0x80, 0x22}, rtp_timestamp, sequence_number++);
  dp.FinishFrame();

  EXPECT_EQ(fallback, 3);
  EXPECT_EQ(cancelled, 0);

  // None of the dropped access units emitted the sprop prefix, so the first
  // real one must still carry it.
  ASSERT_EQ(access_units.size(), 1U);
  const std::vector<std::uint8_t> expected = {
    0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01,
    0x68, 0xCE, 0x3C, 0x80, 0x00, 0x00, 0x00, 0x01, 0x41, 0x80, 0x22};
  EXPECT_EQ(access_units.front(), expected);
}

namespace
{
  void
  append_u32(std::vector<std::uint8_t>& v, std::uint32_t x)
  {
    v.push_back(static_cast<std::uint8_t>(x & 0xFF));
    v.push_back(static_cast<std::uint8_t>((x >> 8) & 0xFF));
    v.push_back(static_cast<std::uint8_t>((x >> 16) & 0xFF));
    v.push_back(static_cast<std::uint8_t>((x >> 24) & 0xFF));
  }

  void
  append_u64(std::vector<std::uint8_t>& v, std::uint64_t x)
  {
    append_u32(v, static_cast<std::uint32_t>(x & 0xFFFFFFFF));
    append_u32(v, static_cast<std::uint32_t>((x >> 32) & 0xFFFFFFFF));
  }

  void
  append_f32(std::vector<std::uint8_t>& v, float f)
  {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(bits));
    append_u32(v, bits);
  }

  // Build the 308-byte RGB_INFO field block (excluding UUID).
  std::vector<std::uint8_t>
  build_rgb_info_body()
  {
    std::vector<std::uint8_t> body;
    append_u32(body, 7);         // version
    append_u32(body, 42);        // frame_counter
    append_u64(body, 123456789); // timestamp_ns
    append_f32(body, 1.5F);      // exposure_time
    for (int i = 0; i < 6; ++i)
      {
        append_f32(body, static_cast<float>(i));
      }
    append_u32(body, 11); // intrinsic model id
    for (int i = 0; i < 32; ++i)
      {
        append_f32(body, static_cast<float>(i) + 0.5F);
      }
    append_u32(body, 22); // inverse intrinsic model id
    for (int i = 0; i < 32; ++i)
      {
        append_f32(body, static_cast<float>(i) + 1.5F);
      }
    return body;
  }
} // namespace

TEST(H264Depacketizer, FallbackVideoAccessUnitStillReportsItsRgbInfo)
{
  H264Depacketizer dp;
  int fallback = 0;
  std::vector<std::uint32_t> frame_counters;
  std::vector<std::vector<std::uint8_t>> access_units;
  dp.on_fallback_access_unit = [&] { ++fallback; };
  dp.on_access_unit = [&](const std::vector<std::uint8_t>& au, std::uint64_t) {
    access_units.push_back(au);
  };
  dp.on_sei_unregistered_user_data =
    [&](const std::array<std::uint8_t, 16>& uuid,
        const std::vector<std::uint8_t>& data) {
      if (const auto info = parse_rgb_info(uuid, data))
        {
          frame_counters.push_back(info->frame_counter);
        }
    };

  // A fallback frame carries RGB_INFO like any other. The client parks that
  // SEI for the access unit it arrived in, so the drop has to be signalled
  // or the metadata leaks onto the next frame.
  const std::vector<std::uint8_t> body = build_rgb_info_body();
  std::vector<std::uint8_t> sei = {0x06, 0x05, 0xFF};
  sei.push_back(static_cast<std::uint8_t>(16 + body.size() - 0xFF));
  sei.insert(sei.end(), RGB_INFO_UUID.begin(), RGB_INFO_UUID.end());
  sei.insert(sei.end(), body.begin(), body.end());
  sei.push_back(0x05); // second message: payloadType 5
  sei.push_back(0x10); // payloadSize 16 (UUID only)
  sei.insert(sei.end(),
             FALLBACK_VIDEO_UUID.begin(),
             FALLBACK_VIDEO_UUID.end());
  sei.push_back(0x80);

  dp.DecodePackage(sei, 9000, 1);
  dp.DecodePackage({0x41, 0x80, 0x11}, 9000, 2);
  dp.FinishFrame();

  EXPECT_TRUE(access_units.empty());
  EXPECT_EQ(fallback, 1);
  ASSERT_EQ(frame_counters.size(), 1U)
    << "the RGB_INFO of a fallback frame is still reported, so the consumer "
       "needs the drop signal to discard it";
  EXPECT_EQ(frame_counters.front(), 42U);
}

// ---------------------------------------------------------------------------
// SEI / RGB_INFO parsing
// ---------------------------------------------------------------------------

TEST(SeiParser, ExtractsRgbInfoFrameIdentity)
{
  const std::vector<std::uint8_t> body = build_rgb_info_body();
  ASSERT_EQ(body.size(), RGB_INFO_WIRE_SIZE);

  const auto info = parse_rgb_info(RGB_INFO_UUID, body);
  ASSERT_TRUE(info.has_value());
  if (!info.has_value())
    {
      return;
    }
  EXPECT_EQ(info->frame_counter, 42U);
  EXPECT_EQ(info->timestamp_ns, 123456789ULL);
  EXPECT_EQ(info->data, body);
}

TEST(SeiParser, RgbInfoPayloadIsCompatibleWithRgbInfoV1)
{
  std::vector<std::uint8_t> body = build_rgb_info_body();
  body.push_back(0xAA);

  const auto payload = parse_rgb_info(RGB_INFO_UUID, body);
  if (!payload.has_value())
    {
      FAIL() << "RGB_INFO payload was rejected";
      return;
    }
  const auto& data = payload->data;
  ASSERT_EQ(data.size(), ifm3d::RGBInfoV1::RGB_INFO_V1_SIZE);

  ifm3d::Buffer buffer(static_cast<std::uint32_t>(data.size()),
                       1,
                       1,
                       ifm3d::PixelFormat::FORMAT_8U,
                       std::nullopt,
                       ifm3d::buffer_id::RGB_INFO);
  std::copy(data.begin(), data.end(), buffer.Ptr<std::uint8_t>(0));

  const auto info = ifm3d::RGBInfoV1::Deserialize(buffer);
  EXPECT_EQ(info.version, 7U);
  EXPECT_EQ(info.frame_counter, 42U);
  EXPECT_EQ(info.timestamp_ns, 123456789ULL);
  EXPECT_FLOAT_EQ(info.exposure_time, 1.5F);

  const auto deserialized = ifm3d::deserialize(buffer);
  ASSERT_TRUE(std::holds_alternative<ifm3d::RGBInfoV1>(deserialized));
}

TEST(SeiParser, RejectsWrongUuid)
{
  const std::vector<std::uint8_t> body = build_rgb_info_body();
  std::array<std::uint8_t, 16> const uuid{};
  EXPECT_FALSE(parse_rgb_info(uuid, body).has_value());
}

TEST(SeiParser, ExtractsUnregisteredUserDataFromSeiNal)
{
  // SEI NAL payload (after NAL header): type=5, size=16+308=324, uuid, body.
  const std::vector<std::uint8_t> body = build_rgb_info_body();
  const std::size_t msg_size = 16 + body.size();

  std::vector<std::uint8_t> sei;
  sei.push_back(0x05); // payloadType = 5
  // payloadSize (324) encoded as runs of 0xFF then remainder.
  std::size_t remaining = msg_size;
  while (remaining >= 0xFF)
    {
      sei.push_back(0xFF);
      remaining -= 0xFF;
    }
  sei.push_back(static_cast<std::uint8_t>(remaining));
  sei.insert(sei.end(), RGB_INFO_UUID.begin(), RGB_INFO_UUID.end());
  sei.insert(sei.end(), body.begin(), body.end());
  sei.push_back(0x80); // RBSP stop bit

  bool got = false;
  parse_sei_nal(sei.data(),
                static_cast<int>(sei.size()),
                [&](const std::array<std::uint8_t, 16>& uuid,
                    const std::vector<std::uint8_t>& data) {
                  const auto info = parse_rgb_info(uuid, data);
                  if (info)
                    {
                      got = true;
                      EXPECT_EQ(info->frame_counter, 42U);
                    }
                });
  EXPECT_TRUE(got);
}

// ---------------------------------------------------------------------------
// Decoder host NAL-only fallback (no decoders available)
// ---------------------------------------------------------------------------

TEST(DecoderHost, NoDecoderRunsInNalOnlyMode)
{
  // With no real decoder available the "null" decoder is used, which
  // discards data and never produces a frame (NAL-only mode).
  DecoderHost host(std::nullopt, false, false);
  EXPECT_TRUE(host.Init());
  EXPECT_FALSE(host.ProducesFrames());

  bool frame_seen = false;
  host.on_frame = [&](const BufferMap&, std::uint64_t) { frame_seen = true; };
  const std::vector<std::uint8_t> au = {0x00, 0x00, 0x00, 0x01, 0x41};
  host.SubmitAccessUnit(au, 0);
  EXPECT_FALSE(frame_seen);
}

TEST(DecoderHost, NullDecoderDoesNotReceiveAccessUnits)
{
  DecoderHost host("null", true, false);
  ASSERT_TRUE(host.Init());
  ASSERT_FALSE(host.ProducesFrames());

  bool error_seen = false;
  host.on_error = [&](int, const std::string&) { error_seen = true; };
  host.SubmitAccessUnit({0x00, 0x00, 0x00, 0x01, 0x41}, 0);
  EXPECT_FALSE(error_seen);
}

// ---------------------------------------------------------------------------
// Frame / access unit pairing across decoder lag
// ---------------------------------------------------------------------------

namespace
{
  // A tiny 16x16 constrained-baseline H.264 stream -- SPS+PPS+IDR followed by
  // five P slices -- whose SPS VUI advertises max_num_reorder_frames = 2.
  //
  // That mirrors what the O3R/O3C actually sends: the profile forbids B
  // slices, yet the SPS still asks decoders to reorder, and libavcodec obeys
  // by holding every frame back for two access units. Pairing a decoded image
  // with "the access unit that was submitted last" therefore mislabels every
  // frame in the stream, which is what this data is here to catch.
  const std::vector<std::uint8_t> AU0 = {
    0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0xC0, 0x0A, 0xD9, 0x1E, 0x80, 0x6D,
    0x04, 0x42, 0x2C, 0xB0, 0x00, 0x00, 0x00, 0x01, 0x68, 0xCB, 0x83, 0xCB,
    0x20, 0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x11, 0xFB, 0x05, 0xE8, 0x12,
    0x0D, 0x83, 0x80, 0x03, 0xE9, 0x6F, 0xF2, 0x41, 0x62, 0x81, 0xC2, 0xFF,
    0x6D, 0x58, 0xD8, 0xB6, 0x14, 0x42, 0x90, 0xD7, 0xF7, 0x88, 0x84, 0x44,
    0xAB, 0xCC, 0x2A, 0x7B, 0x74, 0x9E, 0x0E, 0xE2, 0x30, 0xDE, 0x94, 0x52,
    0x78, 0x28, 0xDD, 0xAE, 0x0B, 0x95, 0x99, 0x62, 0x15, 0x91, 0x41, 0x8D,
    0x67, 0xFD, 0xFE, 0x69, 0xE4, 0x64, 0x70, 0x79, 0x50, 0x4B, 0x34, 0x1B,
    0x80, 0x17, 0xE1, 0x24, 0xA7, 0x00, 0x78, 0xBE, 0xFD, 0x0A, 0xCA, 0x32,
    0x1B, 0x05, 0x1C, 0x8C, 0xAE, 0x0C, 0x51, 0xDB, 0x21, 0x01, 0xEB, 0xDA,
    0xEB, 0xD5, 0xED, 0x59, 0x80, 0x04, 0xF6, 0x0C, 0x00, 0x0B, 0x00, 0xD2,
    0x09, 0xC4, 0x00, 0x68, 0x86, 0xDA, 0xE6, 0x15, 0xE7, 0x86, 0xC5, 0x90,
    0x9A, 0xDA, 0xC6, 0x10, 0xF5, 0x73, 0x00, 0xF9, 0x69, 0x15, 0x80, 0x63,
    0x55, 0x75, 0x88, 0x26, 0xF2, 0x52, 0x91, 0xBF, 0x77, 0x80, 0x03, 0xCE,
    0x05, 0x17, 0x0B, 0x55, 0x8A, 0xEA, 0x16, 0xBF, 0xAE, 0x07, 0x02, 0x9B,
    0x55, 0x3D, 0x83, 0xA9, 0x8A, 0xDF, 0x7A, 0xE6, 0x00, 0x2E, 0x6A, 0xB0,
    0x18, 0x5C, 0xA7, 0xC7, 0xFE, 0xD1, 0xB0, 0x01, 0x21, 0x73, 0x70, 0x48,
    0xCD, 0xC1, 0xAD};

  const std::vector<std::uint8_t> AU1 = {
    0x00, 0x00, 0x00, 0x01, 0x41, 0x9A, 0x38, 0x14, 0xFE, 0x18, 0xE1,
    0x36, 0x47, 0x82, 0x2E, 0xFE, 0xB4, 0xA3, 0xB9, 0xA9, 0xF0};

  const std::vector<std::uint8_t> AU2 = {
    0x00, 0x00, 0x00, 0x01, 0x41, 0x9A, 0x54, 0x04, 0x3F, 0xC3, 0x1C, 0x26,
    0x2B, 0x31, 0xF1, 0xC0, 0x99, 0xCF, 0xF8, 0xB0, 0x50, 0xC2, 0x31, 0x1C,
    0x24, 0x7B, 0x11, 0x9E, 0xB1, 0x61, 0x21, 0xF0, 0x15, 0x6A, 0x20};

  const std::vector<std::uint8_t> AU3 = {
    0x00, 0x00, 0x00, 0x01, 0x41, 0x9A, 0x60, 0x67, 0xF8, 0x63, 0x81,
    0x09, 0x8A, 0x7E, 0x4B, 0x56, 0xE4, 0x30, 0x2D, 0x79, 0xF3, 0x93,
    0xA8, 0xE0, 0x92, 0x8E, 0x11, 0x9F, 0x38, 0x43, 0xF5, 0xEA, 0x60,
    0x26, 0x6A, 0xC3, 0xD9, 0xB3, 0xBE, 0x4F, 0x9A};

  const std::vector<std::uint8_t> AU4 = {
    0x00, 0x00, 0x00, 0x01, 0x41, 0x9A, 0x80, 0x57, 0xF8, 0x62, 0xF0,
    0x15, 0xC5, 0xC9, 0x81, 0x3D, 0xA8, 0x8B, 0xBD, 0xEA, 0xAE, 0x0F,
    0x24, 0x19, 0xDD, 0x08, 0xC2, 0x27, 0x1C, 0x01, 0x97, 0x57, 0xF3,
    0xE4, 0xFC, 0xB6, 0x61, 0x35, 0xDC, 0x88, 0x6E, 0xAC, 0x40};

  const std::vector<std::uint8_t> AU5 = {
    0x00, 0x00, 0x00, 0x01, 0x41, 0x9A, 0xA0, 0x47, 0xF8, 0x63, 0x80, 0x88,
    0xAB, 0x52, 0xF7, 0x1A, 0xC8, 0xEA, 0xDA, 0x53, 0xE7, 0xBE, 0xCD, 0xB9,
    0x94, 0xF4, 0x0D, 0x8E, 0xE0, 0x4B, 0xDB, 0x88, 0xC2, 0xAE, 0x51, 0x55,
    0x80, 0x18, 0xFF, 0x54, 0xD3, 0x0C, 0x9E, 0xBD, 0x4F, 0xF2};
} // namespace

TEST(DecoderHost, PairsEachFrameWithItsOwnAccessUnit)
{
  DecoderHost host(std::nullopt, true, false);
  ASSERT_TRUE(host.Init());
  if (!host.ProducesFrames())
    {
      GTEST_SKIP() << "no real H.264 decoder available";
    }

  std::vector<std::uint64_t> decoded_pts;
  host.on_frame = [&](const BufferMap&, std::uint64_t pts) {
    decoded_pts.push_back(pts);
  };

  const std::vector<std::vector<std::uint8_t>> access_units =
    {AU0, AU1, AU2, AU3, AU4, AU5};

  // Offset the pts so that a frame accidentally labelled with an access unit
  // index rather than its pts cannot pass.
  constexpr std::uint64_t pts_base = 1000;
  for (std::size_t i = 0; i < access_units.size(); ++i)
    {
      host.SubmitAccessUnit(access_units[i], pts_base + i);
    }

  ASSERT_FALSE(decoded_pts.empty()) << "decoder produced no frames at all";

  // A decoder that emits every access unit immediately is equally correct
  // (video_decoder.h permits lag, it does not require it), and the pairing
  // checks below still hold for it, so this is reported rather than asserted.
  if (decoded_pts.size() == access_units.size())
    {
      GTEST_LOG_(INFO) << "decoder buffered nothing; the reordering path is "
                          "not exercised on this build";
    }

  // Whatever did surface must carry the pts of the access unit it was decoded
  // from, not of the one that happened to be submitted when it came out.
  for (std::size_t i = 0; i < decoded_pts.size(); ++i)
    {
      EXPECT_EQ(decoded_pts[i], pts_base + i)
        << "frame " << i << " was paired with the wrong access unit";
    }
}

// ---------------------------------------------------------------------------
// UDP transport shutdown
// ---------------------------------------------------------------------------

// Regression test for the UDP shutdown deadlock: RtpConnectionUdp arms
// self-perpetuating async operations (the RTP/RTCP receive re-arms on every
// completion, and the NAT-piercing watchdog timer re-arms itself). Merely
// releasing the io_context work guard therefore does NOT let run() return, so
// RtspClient::Stop() must call io_context::stop() to break the loop before
// joining its worker thread.
TEST(RtpConnectionUdp, StopBreaksSelfArmingReceiveLoop)
{
  asio::io_context ctx;
  auto guard = asio::make_work_guard(ctx);

  // Constructing the connection arms an outstanding async_receive_from;
  // enabling the watchdog additionally arms the re-arming piercing timer.
  auto conn = std::make_shared<RtpConnectionUdp>(ctx, /*local_port=*/0);
  conn->EnablePiercingWatchDog();

  auto run_result = std::async(std::launch::async, [&ctx]() { ctx.run(); });

  // Releasing the work guard is not sufficient: the self-arming UDP operations
  // keep run() blocked, so the worker would never finish (the original bug).
  asio::post(ctx, [&guard]() { guard.reset(); });
  EXPECT_EQ(run_result.wait_for(std::chrono::milliseconds(300)),
            std::future_status::timeout)
    << "self-arming UDP operations should keep io_context.run() busy";

  // stop() is what RtspClient::Stop() now does; it must make run() return.
  ctx.stop();
  ASSERT_EQ(run_result.wait_for(std::chrono::seconds(5)),
            std::future_status::ready)
    << "io_context.run() did not return after stop(); UDP shutdown deadlock";
  run_result.get();
}

// ---------------------------------------------------------------------------
// RTSP handshake failure detection
// ---------------------------------------------------------------------------

namespace
{
  /* Minimal RTSP server that replies to the requests it receives with a
   * canned script of responses and then closes the connection. Serving fewer
   * responses than the client sends requests models a server that drops the
   * connection mid-handshake.
   *
   * Every step is asynchronous on purpose: a synchronous accept()/read_some()
   * cannot be woken from another thread, so a client that never connects (or
   * never sends) would wedge the destructor's join() and turn a test failure
   * into a silent hang. The destructor instead posts the shutdown into the
   * io_context, and a watchdog timer bounds the server's lifetime even if the
   * destructor is never reached. */
  class ScriptedRtspServer
  {
  public:
    explicit ScriptedRtspServer(std::vector<std::string> responses)
      : _acceptor(
          _ctx,
          asio::ip::tcp::endpoint(asio::ip::address_v4::loopback(), 0)),
        _socket(_ctx),
        _watchdog(_ctx),
        _responses(std::move(responses))
    {
      _port = _acceptor.local_endpoint().port();
      arm_watchdog();
      accept();
      _thread = std::thread([this]() { _ctx.run(); });
    }

    ~ScriptedRtspServer()
    {
      asio::post(_ctx, [this]() { shutdown(); });
      if (_thread.joinable())
        {
          _thread.join();
        }
    }

    ScriptedRtspServer(const ScriptedRtspServer&) = delete;
    ScriptedRtspServer& operator=(const ScriptedRtspServer&) = delete;
    ScriptedRtspServer(ScriptedRtspServer&&) = delete;
    ScriptedRtspServer& operator=(ScriptedRtspServer&&) = delete;

    [[nodiscard]] std::uint16_t
    Port() const
    {
      return _port;
    }

  private:
    static constexpr std::chrono::seconds WATCHDOG_TIMEOUT{10};

    // Drops every outstanding operation so io_context::run() runs out of work
    // and the server thread finishes.
    void
    shutdown()
    {
      asio::error_code ec;
      _watchdog.cancel();
      std::ignore = _acceptor.close(ec);
      std::ignore = _socket.close(ec);
    }

    void
    arm_watchdog()
    {
      _watchdog.expires_after(WATCHDOG_TIMEOUT);
      _watchdog.async_wait([this](const asio::error_code& ec) {
        if (!ec)
          {
            shutdown();
          }
      });
    }

    void
    accept()
    {
      _acceptor.async_accept(_socket, [this](const asio::error_code& ec) {
        if (ec)
          {
            shutdown();
            return;
          }
        read_request();
      });
    }

    void
    read_request()
    {
      if (_next_response >= _responses.size())
        {
          // Out of scripted responses: drop the connection, which is what the
          // "server disappears mid-handshake" cases exercise.
          shutdown();
          return;
        }

      _socket.async_read_some(
        asio::buffer(_buffer),
        [this](const asio::error_code& ec, std::size_t read) {
          if (ec)
            {
              shutdown();
              return;
            }
          _request.append(_buffer.data(), read);
          if (_request.find("\r\n\r\n") == std::string::npos)
            {
              read_request();
              return;
            }
          _request.clear();
          write_response();
        });
    }

    void
    write_response()
    {
      const std::string& response = _responses.at(_next_response);
      _socket.async_write_some(
        asio::buffer(response.data() + _written, response.size() - _written),
        [this](const asio::error_code& ec, std::size_t written) {
          if (ec)
            {
              shutdown();
              return;
            }
          _written += written;
          if (_written < _responses.at(_next_response).size())
            {
              write_response();
              return;
            }
          _written = 0;
          ++_next_response;
          read_request();
        });
    }

    asio::io_context _ctx;
    asio::ip::tcp::acceptor _acceptor;
    asio::ip::tcp::socket _socket;
    asio::steady_timer _watchdog;
    std::vector<std::string> _responses;
    std::size_t _next_response = 0;
    std::size_t _written = 0;
    std::string _request;
    std::array<char, 1024> _buffer{};
    std::uint16_t _port = 0;
    std::thread _thread;
  };

  const std::string SDP_BODY = "v=0\r\n"
                               "o=- 0 0 IN IP4 127.0.0.1\r\n"
                               "s=Session\r\n"
                               "m=video 0 RTP/AVP 96\r\n"
                               "a=rtpmap:96 H264/90000\r\n"
                               "a=control:trackID=0\r\n";

  std::string
  describe_ok()
  {
    return "RTSP/1.0 200 OK\r\nCSeq: 1\r\nContent-Type: application/sdp\r\n"
           "Content-Length: " +
           std::to_string(SDP_BODY.size()) + "\r\n\r\n" + SDP_BODY;
  }

  const std::string SETUP_OK =
    "RTSP/1.0 200 OK\r\nCSeq: 2\r\nSession: 12345678;timeout=60\r\n"
    "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n";

  /* Drives a session against the scripted server until it settles and returns
   * the reported error codes (in order), or an empty list if none was
   * reported. */
  struct HandshakeOutcome
  {
    std::vector<int> errors;
    bool playing = false;

    [[nodiscard]] std::optional<int>
    FirstError() const
    {
      return errors.empty() ? std::nullopt :
                              std::optional<int>(errors.front());
    }
  };

  HandshakeOutcome
  run_handshake(std::uint16_t port)
  {
    asio::io_context ctx;
    RtspSession session(ctx);

    HandshakeOutcome outcome;
    session.on_error = [&outcome](int code) {
      outcome.errors.push_back(code);
    };
    session.on_playing = [&outcome]() { outcome.playing = true; };

    session.InitConnection("127.0.0.1",
                           port,
                           RtspSession::TransportType::INTERLEAVED,
                           "port1");
    ctx.run_for(std::chrono::seconds(5));
    return outcome;
  }
} // namespace

// The scripted server must be destructible even when no client ever connects:
// a destructor that joins a thread parked in a synchronous accept() would turn
// any such test failure into an indefinite CI hang instead of a red test.
TEST(ScriptedRtspServer, DestructorDoesNotBlockWithoutAClient)
{
  auto destroyed = std::make_shared<std::promise<void>>();
  auto destroyed_future = destroyed->get_future();

  std::thread worker([destroyed]() {
    auto server = std::make_unique<ScriptedRtspServer>(
      std::vector<std::string>{describe_ok()});
    // Give the server thread time to park in its accept operation.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    server.reset();
    destroyed->set_value();
  });

  const bool destructed = destroyed_future.wait_for(std::chrono::seconds(5)) ==
                          std::future_status::ready;
  if (destructed)
    {
      worker.join();
    }
  else
    {
      // Only reached once the regression is back. Joining would hang the whole
      // test binary, which is exactly the failure mode this test exists to
      // prevent, so the stuck thread is deliberately abandoned instead.
      worker.detach();
    }

  EXPECT_TRUE(destructed)
    << "~ScriptedRtspServer() blocked although no client connected";
}

// A DESCRIBE that the server rejects (e.g. the port is not streaming yet)
// must be reported; the handshake must not be left pending.
TEST(RtspSession, DescribeFailureIsReported)
{
  const ScriptedRtspServer server(
    {"RTSP/1.0 404 Not Found\r\nCSeq: 1\r\n\r\n"});

  const HandshakeOutcome outcome = run_handshake(server.Port());

  ASSERT_TRUE(outcome.FirstError().has_value());
  EXPECT_EQ(outcome.FirstError().value_or(0), IFM3D_RTSP_REQUEST_FAILED);
  EXPECT_FALSE(outcome.playing);
}

// The PLAY response status used to be ignored, so a rejected PLAY still
// reported the session as playing.
TEST(RtspSession, PlayFailureIsReported)
{
  const ScriptedRtspServer server(
    {describe_ok(),
     SETUP_OK,
     "RTSP/1.0 455 Method Not Valid In This State\r\nCSeq: 3\r\n"
     "Session: 12345678\r\n\r\n"});

  const HandshakeOutcome outcome = run_handshake(server.Port());

  ASSERT_TRUE(outcome.FirstError().has_value());
  EXPECT_EQ(outcome.FirstError().value_or(0), IFM3D_RTSP_REQUEST_FAILED);
  EXPECT_FALSE(outcome.playing);
}

// A SETUP rejected as unsupported transport keeps its dedicated error code.
TEST(RtspSession, UnsupportedTransportIsReported)
{
  const ScriptedRtspServer server(
    {describe_ok(), "RTSP/1.0 461 Unsupported Transport\r\nCSeq: 2\r\n\r\n"});

  const HandshakeOutcome outcome = run_handshake(server.Port());

  ASSERT_TRUE(outcome.FirstError().has_value());
  EXPECT_EQ(outcome.FirstError().value_or(0),
            IFM3D_RTSP_TRANSPORT_UNSUPPORTED);
  EXPECT_FALSE(outcome.playing);
}

// A server that accepts the connection and then drops it without answering
// used to be silent, leaving the caller's Start() future unresolved forever.
TEST(RtspSession, ConnectionLossDuringHandshakeIsReported)
{
  const ScriptedRtspServer server({});

  const HandshakeOutcome outcome = run_handshake(server.Port());

  ASSERT_TRUE(outcome.FirstError().has_value());
  EXPECT_EQ(outcome.FirstError().value_or(0), IFM3D_RTSP_CONNECTION_ERROR);
  EXPECT_FALSE(outcome.playing);
}

// An unparseable response is already reported by the message parser; the
// status check must not report it a second time with a less accurate code.
TEST(RtspSession, MalformedResponseIsReportedOnce)
{
  const ScriptedRtspServer server({"NOT-AN-RTSP-RESPONSE\r\n\r\n"});

  const HandshakeOutcome outcome = run_handshake(server.Port());

  EXPECT_EQ(outcome.errors, std::vector<int>{IFM3D_RTSP_ERROR});
  EXPECT_FALSE(outcome.playing);
}

// ---------------------------------------------------------------------------
// Port parsing (server_port robustness)
// ---------------------------------------------------------------------------

TEST(ParsePort, AcceptsValidPorts)
{
  EXPECT_EQ(parse_port("0"), 0);
  EXPECT_EQ(parse_port("554"), 554);
  EXPECT_EQ(parse_port("8554"), 8554);
  EXPECT_EQ(parse_port("65535"), 65535);
}

TEST(ParsePort, RejectsOutOfRangeAndMalformed)
{
  // A value above 65535 must yield 0 rather than throwing or truncating; this
  // is the SETUP server_port hardening (a hostile server sending e.g.
  // "server_port=99999999999-..." must not abort the session).
  EXPECT_EQ(parse_port("65536"), 0);
  EXPECT_EQ(parse_port("70000"), 0);
  EXPECT_EQ(parse_port("99999999999"), 0);
  EXPECT_EQ(parse_port("4294967296"), 0); // overflows uint32
  EXPECT_EQ(parse_port(""), 0);
  EXPECT_EQ(parse_port("abc"), 0);
  EXPECT_EQ(parse_port("12ab"), 0); // trailing non-digits
  EXPECT_EQ(parse_port("-1"), 0);
}

// ---------------------------------------------------------------------------
// RTP header parsing (extension handling)
// ---------------------------------------------------------------------------

namespace
{
  // Captures the payloads handed to DecodePackage.
  class CapturingPackageDecoder : public PackageDecoder
  {
  public:
    void
    DecodePackage(const std::vector<std::uint8_t>& package,
                  std::uint32_t,
                  std::uint16_t) override
    {
      packages.push_back(package);
    }
    void
    FinishFrame() override
    {}
    void
    CancelFrame() override
    {}

    std::vector<std::vector<std::uint8_t>> packages;
  };

  // Minimal in-memory RTP transport: packets are injected via
  // on_data_received.
  class MockRtpConnection : public RtpConnection
  {
  public:
    void
    Send(ByteSpan) override
    {}
  };

  // Build a 12-byte RTP fixed header.
  std::vector<std::uint8_t>
  make_rtp_header(bool ext,
                  std::uint8_t csrc_count,
                  bool marker,
                  std::uint8_t payload_type,
                  std::uint16_t seq)
  {
    std::vector<std::uint8_t> h(12, 0);
    h[0] = static_cast<std::uint8_t>((2U << 6) | (ext ? 0x10U : 0U) |
                                     (csrc_count & 0x0FU));
    h[1] = static_cast<std::uint8_t>((marker ? 0x80U : 0U) |
                                     (payload_type & 0x7FU));
    h[2] = static_cast<std::uint8_t>((seq >> 8) & 0xFF);
    h[3] = static_cast<std::uint8_t>(seq & 0xFF);
    return h;
  }
} // namespace

TEST(RtpClient, SkipsRtpExtensionHeaderWhenComputingPayload)
{
  auto conn = std::make_shared<MockRtpConnection>();
  RtpClient client;
  client.InitConnection(conn);
  auto decoder = std::make_shared<CapturingPackageDecoder>();
  client.RegisterDecoder(96, decoder);

  // Prime _frame_valid: a marker packet is not itself decoded, but flips the
  // client into the "frame valid" state so the next packet is dispatched.
  auto prime = make_rtp_header(/*ext=*/false, 0, /*marker=*/true, 96, 100);
  prime.push_back(0x00);
  conn->on_data_received(prime);
  ASSERT_TRUE(decoder->packages.empty());

  // Next in-sequence packet carries an RTP extension: a 4-byte extension
  // header (profile 0xBEDE + length = 2 words) followed by 8 bytes of
  // extension data, then the real payload.
  auto pkt = make_rtp_header(/*ext=*/true, 0, /*marker=*/false, 96, 101);
  pkt.push_back(0xBE);
  pkt.push_back(0xDE);
  pkt.push_back(0x00);
  pkt.push_back(0x02); // 2 words follow
  for (int i = 0; i < 8; ++i)
    {
      pkt.push_back(0x11);
    }
  const std::vector<std::uint8_t> payload = {0xAA, 0xBB, 0xCC, 0xDD};
  pkt.insert(pkt.end(), payload.begin(), payload.end());

  conn->on_data_received(pkt);

  ASSERT_EQ(decoder->packages.size(), 1U);
  EXPECT_EQ(decoder->packages[0], payload)
    << "RTP extension words must be skipped, not fed to the depacketizer";
}

TEST(RtpClient, DiscardsTruncatedExtensionHeader)
{
  auto conn = std::make_shared<MockRtpConnection>();
  RtpClient client;
  client.InitConnection(conn);
  auto decoder = std::make_shared<CapturingPackageDecoder>();
  client.RegisterDecoder(96, decoder);

  auto prime = make_rtp_header(/*ext=*/false, 0, /*marker=*/true, 96, 100);
  prime.push_back(0x00);
  conn->on_data_received(prime);

  // Extension bit set but fewer than 4 bytes of extension header present.
  auto pkt = make_rtp_header(/*ext=*/true, 0, /*marker=*/false, 96, 101);
  pkt.push_back(0xBE);
  pkt.push_back(0xDE); // only 2 of the 4 required extension-header bytes

  EXPECT_NO_THROW(conn->on_data_received(pkt));
  EXPECT_TRUE(decoder->packages.empty())
    << "a truncated RTP extension header must be discarded, not parsed";
}

TEST(RtpClient, PacketLossIsVisibleInNalUnitSequenceNumbers)
{
  auto conn = std::make_shared<MockRtpConnection>();
  RtpClient client;
  client.InitConnection(conn);
  auto depacketizer = std::make_shared<H264Depacketizer>();
  client.RegisterDecoder(96, depacketizer);

  std::vector<ifm3d::NalUnit> nals;
  depacketizer->on_nal_unit = [&](const ifm3d::NalUnit& n) {
    nals.push_back(n);
  };

  const auto send = [&](std::uint16_t seq, bool marker, std::uint8_t payload) {
    auto pkt = make_rtp_header(/*ext=*/false, 0, marker, 96, seq);
    pkt.push_back(0x41); // non-IDR slice NAL header
    pkt.push_back(0x40);
    pkt.push_back(payload);
    conn->on_data_received(pkt);
  };

  send(100, /*marker=*/true, 0x11);  // primes _frame_valid, not decoded
  send(101, /*marker=*/false, 0x22); // delivered
  // 102..104 lost; 105 carries the marker that cancels the access unit
  send(105, /*marker=*/true, 0x33);
  send(106, /*marker=*/false, 0x44); // delivered

  // A consumer of the public on_nal_unit callback receives NALs of an access
  // unit that was subsequently abandoned. The discontinuity is observable
  // without any additional signal, because NalUnit carries the RTP sequence
  // numbers the NAL was assembled from.
  ASSERT_EQ(nals.size(), 2U);
  EXPECT_EQ(nals[0].last_sequence_number, 101);
  EXPECT_EQ(nals[1].first_sequence_number, 106);
  EXPECT_NE(static_cast<std::uint16_t>(nals[1].first_sequence_number -
                                       nals[0].last_sequence_number),
            1)
    << "packet loss must leave a gap in the exposed sequence numbers";
}
