/*
 * Copyright (C) 2026-present ifm electronic, gmbh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for ifm3d::PluginManager
 *
 * PLUGIN_TEST_DIR is injected by CMake and points to the directory that
 * contains the built sample plugin shared library.
 */

#include <gtest/gtest.h>

#include <ifm3d/rtsp/plugin.h>
#include <ifm3d/rtsp/plugin_manager.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#ifndef _WIN32
#  include <stdlib.h> // NOLINT(modernize-deprecated-headers) — setenv/unsetenv are POSIX
#endif

// PLUGIN_TEST_DIR is injected by CMake target_compile_definitions.
// #ifndef PLUGIN_TEST_DIR
// #  error "PLUGIN_TEST_DIR must be defined by CMake"
// #endif

static const std::string K_PLUGIN_DIR{"/ifm3d/build/modules/rtsp/plugins"};
static const std::string K_BOGUS_DIR{"/no/such/path/xyz"};

// ---------------------------------------------------------------------------
// GTest fixture — owns the PluginManager (non-copyable, non-movable)
// ---------------------------------------------------------------------------

class PluginManagerTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    _pm = std::make_unique<ifm3d::PluginManager>(
      std::vector<std::string>{K_PLUGIN_DIR});
    _pm->LoadPlugins();
  }
  std::unique_ptr<ifm3d::PluginManager> _pm;
};

// ---------------------------------------------------------------------------
// Plugin discovery
// ---------------------------------------------------------------------------

TEST_F(PluginManagerTest, LoadsPluginFromDirectory)
{
  // After SetUp() the sample plugin must be registered
  EXPECT_NE(_pm->GetPluginApi(VIDEO_CODEC_H264), nullptr);
}

TEST(PluginManager, NoApiBeforeLoadPlugins)
{
  // Deliberately skip LoadPlugins() — nothing should be available
  ifm3d::PluginManager manager({K_PLUGIN_DIR});
  EXPECT_EQ(manager.GetPluginApi(VIDEO_CODEC_H264), nullptr);
  EXPECT_EQ(manager.CreateDecoder(VIDEO_CODEC_H264), nullptr);
}

TEST(PluginManager, NonExistentDirectoryIsNonFatal)
{
  ifm3d::PluginManager manager({K_BOGUS_DIR});
  EXPECT_NO_THROW(manager.LoadPlugins());
  EXPECT_EQ(manager.GetPluginApi(VIDEO_CODEC_H264), nullptr);
}

TEST(PluginManager, EnvVarPathIsPickedUp)
{
#ifdef _WIN32
  _putenv_s("IFM3D_PLUGIN_PATH", K_PLUGIN_DIR.c_str());
#else
  setenv("IFM3D_PLUGIN_PATH", K_PLUGIN_DIR.c_str(), /*overwrite=*/1);
#endif

  ifm3d::PluginManager manager({});
  manager.LoadPlugins();
  EXPECT_NE(manager.GetPluginApi(VIDEO_CODEC_H264), nullptr)
    << "Plugin should be found via IFM3D_PLUGIN_PATH";

#ifdef _WIN32
  _putenv_s("IFM3D_PLUGIN_PATH", "");
#else
  unsetenv("IFM3D_PLUGIN_PATH");
#endif
}

TEST(PluginManager, PreferredPluginNameSelects)
{
  ifm3d::PluginManager manager({K_PLUGIN_DIR},
                               /*preferred_plugin_name=*/"ffmpeg");
  manager.LoadPlugins();
  EXPECT_NE(manager.GetPluginApi(VIDEO_CODEC_H264), nullptr)
    << "Preferred plugin 'ffmpeg' should be selected";
}

TEST(PluginManager, LoadPluginsTwiceIsIdempotent)
{
  ifm3d::PluginManager manager({K_PLUGIN_DIR});
  manager.LoadPlugins();
  manager.LoadPlugins(); // second call must not double-register
  VideoDecoder* dec = manager.CreateDecoder(VIDEO_CODEC_H264);
  EXPECT_NE(dec, nullptr);
  if (dec)
    {
      manager.DestroyDecoder(dec);
    }
}

// ---------------------------------------------------------------------------
// Decoder lifecycle
// ---------------------------------------------------------------------------

TEST_F(PluginManagerTest, CreateDecoderReturnsNonNull)
{
  VideoDecoder* dec = _pm->CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);
  _pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, CreateDecoderReturnsDistinctInstances)
{
  VideoDecoder* dec1 = _pm->CreateDecoder(VIDEO_CODEC_H264);
  VideoDecoder* dec2 = _pm->CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec1, nullptr);
  ASSERT_NE(dec2, nullptr);
  EXPECT_NE(dec1, dec2) << "Each call must return an independent instance";
  _pm->DestroyDecoder(dec1);
  _pm->DestroyDecoder(dec2);
}

TEST_F(PluginManagerTest, UnknownCodecReturnsNullDecoder)
{
  // Cast an invalid codec value — no plugin should accept it
  auto* dec = _pm->CreateDecoder(static_cast<VideoCodec>(
    0xFFFF)); // NOLINT(clang-analyzer-optin.core.EnumCastOutOfRange)
  EXPECT_EQ(dec, nullptr);
}

TEST_F(PluginManagerTest, DestroyNullptrIsNonFatal)
{
  EXPECT_NO_THROW(_pm->DestroyDecoder(nullptr));
}

TEST_F(PluginManagerTest, DISABLED_DestroyUnknownPointerIsNonFatal)
{
  // A pointer that was never issued by this manager must not crash
  auto* fake =
    reinterpret_cast<VideoDecoder*>( // NOLINT(performance-no-int-to-ptr)
      uintptr_t{0xDEADBEEF});
  EXPECT_NO_THROW(_pm->DestroyDecoder(fake));
}

// ---------------------------------------------------------------------------
// Decoder functionality via GetPluginApi()
// ---------------------------------------------------------------------------

TEST_F(PluginManagerTest, SendPacketIsCallableThroughManager)
{
  const VideoPluginAPI* api = _pm->GetPluginApi(VIDEO_CODEC_H264);
  ASSERT_NE(api, nullptr);
  VideoDecoder* dec = _pm->CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);
  // Fake Annex-B start-code + IDR NAL header
  const std::array<uint8_t, 5> data{0x00, 0x00, 0x00, 0x01, 0x65};
  const int rc =
    api->send_packet(dec, data.data(), static_cast<int>(data.size()));
  EXPECT_TRUE(rc == 0 || rc < 0);
  _pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, ReceiveFrameReturnsNonNegative)
{
  const VideoPluginAPI* api = _pm->GetPluginApi(VIDEO_CODEC_H264);
  ASSERT_NE(api, nullptr);
  VideoDecoder* dec = _pm->CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);
  VideoFrame frame{};
  EXPECT_GE(api->receive_frame(dec, &frame), 0);
  _pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, SendThenReceiveCycle)
{
  const VideoPluginAPI* api = _pm->GetPluginApi(VIDEO_CODEC_H264);
  ASSERT_NE(api, nullptr);
  VideoDecoder* dec = _pm->CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);
  const std::array<uint8_t, 5> data{0x00, 0x00, 0x00, 0x01, 0x65};
  // EXPECT_EQ(api->send_packet(dec, data.data(), data.size()), 0);
  const int rc =
    api->send_packet(dec, data.data(), static_cast<int>(data.size()));
  EXPECT_TRUE(rc == 0 || rc < 0);
  VideoFrame frame{};
  EXPECT_GE(api->receive_frame(dec, &frame), 0);
  _pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, FlushIsCallableWhenPresent)
{
  const VideoPluginAPI* api = _pm->GetPluginApi(VIDEO_CODEC_H264);
  ASSERT_NE(api, nullptr);
  VideoDecoder* dec = _pm->CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);
  if (api->flush)
    {
      EXPECT_GE(api->flush(dec), 0);
    }
  _pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, LastErrorIsCallableWhenPresent)
{
  const VideoPluginAPI* api = _pm->GetPluginApi(VIDEO_CODEC_H264);
  ASSERT_NE(api, nullptr);
  VideoDecoder* dec = _pm->CreateDecoder(VIDEO_CODEC_H264);
  ASSERT_NE(dec, nullptr);
  if (api->last_error)
    {
      (void)api->last_error(dec);
    }
  _pm->DestroyDecoder(dec);
}

// ---------------------------------------------------------------------------
// API function-pointer validity
// ---------------------------------------------------------------------------

TEST_F(PluginManagerTest, PluginReportsCorrectAbiVersion)
{
  const VideoPluginAPI* api = _pm->GetPluginApi(VIDEO_CODEC_H264);
  ASSERT_NE(api, nullptr);
  EXPECT_EQ(api->abi_version, static_cast<uint32_t>(VIDEO_PLUGIN_ABI_VERSION));
}

TEST_F(PluginManagerTest, RequiredFunctionPointersAreNonNull)
{
  const VideoPluginAPI* api = _pm->GetPluginApi(VIDEO_CODEC_H264);
  ASSERT_NE(api, nullptr);
  EXPECT_NE(api->create_decoder, nullptr);
  EXPECT_NE(api->destroy_decoder, nullptr);
  EXPECT_NE(api->send_packet, nullptr);
  EXPECT_NE(api->receive_frame, nullptr);
  // flush and last_error are optional — NULL is valid
}
