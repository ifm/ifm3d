/*
 * Copyright 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the RTSP client building blocks (no network required):
 * SDP parsing, the H.264 depacketizer, SEI / RGB_INFO parsing, the bit
 * reader and the decoder host's NAL-only fallback.
 */

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <asio/executor_work_guard.hpp>
#include <asio/io_context.hpp>
#include <asio/post.hpp>

#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/rtsp/frame_metadata.h>
#include <ifm3d/rtsp/nal_unit.h>

#include "bit_reader_writer.hpp"
#include "byte_span.hpp"
#include "decoder_host.hpp"
#include "h264_depacketizer.hpp"
#include "package_decoder.hpp"
#include "rtp_client.hpp"
#include "rtp_connection.hpp"
#include "rtp_connection_udp.hpp"
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
  dp.on_access_unit = [&](std::vector<std::uint8_t> au, std::uint64_t) {
    access_units.push_back(std::move(au));
  };

  // A single non-IDR slice NAL (type 1), nal_ref_idc 2 -> header 0x41.
  const std::vector<std::uint8_t> payload = {0x41, 0xAA, 0xBB, 0xCC};
  dp.DecodePackage(payload, 9000, 1);

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
  dp.on_access_unit = [&](std::vector<std::uint8_t> au, std::uint64_t) {
    access_units.push_back(std::move(au));
  };

  // Original NAL: IDR slice (type 5), nal_ref_idc 3 -> header byte 0x65.
  // FU indicator: f=0,nri=3,type=28(FU-A) -> 0x7C.
  // FU header start: s=1,e=0,type=5 -> 0x85. End: s=0,e=1,type=5 -> 0x45.
  const std::vector<std::uint8_t> frag1 = {0x7C, 0x85, 0x11, 0x22};
  const std::vector<std::uint8_t> frag2 = {0x7C, 0x45, 0x33, 0x44};
  dp.DecodePackage(frag1, 18000, 10);
  dp.DecodePackage(frag2, 18000, 11);

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

// ---------------------------------------------------------------------------
// SEI / RGB_INFO parsing
// ---------------------------------------------------------------------------

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

TEST(SeiParser, ParsesRgbInfoRoundTrip)
{
  const std::vector<std::uint8_t> body = build_rgb_info_body();
  ASSERT_EQ(body.size(), ifm3d::RgbInfo::WIRE_SIZE);

  std::array<std::uint8_t, 16> const uuid = ifm3d::RgbInfo::UUID;
  const auto info = parse_rgb_info(uuid, body);
  ASSERT_TRUE(info.has_value());
  if (!info.has_value())
    {
      return;
    }
  EXPECT_EQ(info->version, 7U);
  EXPECT_EQ(info->frame_counter, 42U);
  EXPECT_EQ(info->timestamp_ns, 123456789ULL);
  EXPECT_FLOAT_EQ(info->exposure_time, 1.5F);
  EXPECT_EQ(info->intrinsic.model_id, 11U);
  EXPECT_FLOAT_EQ(info->intrinsic.model_parameters[0], 0.5F);
  EXPECT_EQ(info->inverse_intrinsic.model_id, 22U);
  EXPECT_FLOAT_EQ(info->inverse_intrinsic.model_parameters[0], 1.5F);
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
  sei.insert(sei.end(),
             ifm3d::RgbInfo::UUID.begin(),
             ifm3d::RgbInfo::UUID.end());
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

TEST(DecoderHost, RgbInfoToJsonContainsKeys)
{
  ifm3d::RgbInfo info;
  info.version = 3;
  info.frame_counter = 99;
  info.timestamp_ns = 555;
  info.exposure_time = 2.0F;
  info.intrinsic.model_id = 4;
  info.inverse_intrinsic.model_id = 5;

  const ifm3d::json j = rgb_info_to_json(info);
  EXPECT_EQ(j[ifm3d::frame_metadata::VERSION], 3U);
  EXPECT_EQ(j[ifm3d::frame_metadata::FRAME_COUNTER], 99U);
  EXPECT_EQ(j[ifm3d::frame_metadata::TIMESTAMP_NS], 555U);
  EXPECT_EQ(j[ifm3d::frame_metadata::INTRINSIC_MODEL_ID], 4U);
  EXPECT_EQ(j[ifm3d::frame_metadata::INVERSE_INTRINSIC_MODEL_ID], 5U);
}

// ---------------------------------------------------------------------------
// Decoder host NAL-only fallback (no decoders available)
// ---------------------------------------------------------------------------

TEST(DecoderHost, NoDecoderRunsInNalOnlyMode)
{
  // With no real decoder available the "null" decoder is used, which
  // discards data and never produces a frame (NAL-only mode).
  DecoderHost host(std::nullopt);
  EXPECT_TRUE(host.Init());
  EXPECT_TRUE(host.HasDecoder());

  bool frame_seen = false;
  host.on_frame = [&](const ifm3d::Buffer&) { frame_seen = true; };
  const std::vector<std::uint8_t> au = {0x00, 0x00, 0x00, 0x01, 0x41};
  host.SubmitAccessUnit(au, 0, std::nullopt);
  EXPECT_FALSE(frame_seen);
}

TEST(DecoderHost, ExplicitNullDecoderRunsInNalOnlyMode)
{
  // Requesting the "null" decoder explicitly disables decoding.
  DecoderHost host("null");
  EXPECT_TRUE(host.Init());
  EXPECT_TRUE(host.HasDecoder());

  bool frame_seen = false;
  host.on_frame = [&](const ifm3d::Buffer&) { frame_seen = true; };
  const std::vector<std::uint8_t> au = {0x00, 0x00, 0x00, 0x01, 0x41};
  host.SubmitAccessUnit(au, 0, std::nullopt);
  EXPECT_FALSE(frame_seen);
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
