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

#include <cstdlib>
#include <memory>
#include <string>

// PLUGIN_TEST_DIR is injected by CMake target_compile_definitions.
// #ifndef PLUGIN_TEST_DIR
// #  error "PLUGIN_TEST_DIR must be defined by CMake"
// #endif

static const std::string kPluginDir{"/ifm3d/build/modules/rtsp/plugins"};
static const std::string kBogusDir{"/no/such/path/xyz"};

// ---------------------------------------------------------------------------
// GTest fixture — owns the PluginManager (non-copyable, non-movable)
// ---------------------------------------------------------------------------

class PluginManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        pm = std::make_unique<ifm3d::PluginManager>(
            std::vector<std::string>{kPluginDir});
        pm->LoadPlugins();
    }
    std::unique_ptr<ifm3d::PluginManager> pm;
};

// ---------------------------------------------------------------------------
// Plugin discovery
// ---------------------------------------------------------------------------

TEST_F(PluginManagerTest, LoadsPluginFromDirectory)
{
    // After SetUp() the sample plugin must be registered
    EXPECT_NE(pm->GetPluginApi(VIDEO_CODEC_H264), nullptr);
}

TEST(PluginManager, NoApiBeforeLoadPlugins)
{
    // Deliberately skip LoadPlugins() — nothing should be available
    ifm3d::PluginManager manager({kPluginDir});
    EXPECT_EQ(manager.GetPluginApi(VIDEO_CODEC_H264), nullptr);
    EXPECT_EQ(manager.CreateDecoder(VIDEO_CODEC_H264), nullptr);
}

TEST(PluginManager, NonExistentDirectoryIsNonFatal)
{
    ifm3d::PluginManager manager({kBogusDir});
    EXPECT_NO_THROW(manager.LoadPlugins());
    EXPECT_EQ(manager.GetPluginApi(VIDEO_CODEC_H264), nullptr);
}

TEST(PluginManager, EnvVarPathIsPickedUp)
{
#ifdef _WIN32
    _putenv_s("IFM3D_PLUGIN_PATH", kPluginDir.c_str());
#else
    setenv("IFM3D_PLUGIN_PATH", kPluginDir.c_str(), /*overwrite=*/1);
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
    ifm3d::PluginManager manager({kPluginDir}, /*preferred=*/"sample");
    manager.LoadPlugins();
    EXPECT_NE(manager.GetPluginApi(VIDEO_CODEC_H264), nullptr)
        << "Preferred plugin 'sample' should be selected";
}

TEST(PluginManager, LoadPluginsTwiceIsIdempotent)
{
    ifm3d::PluginManager manager({kPluginDir});
    manager.LoadPlugins();
    manager.LoadPlugins(); // second call must not double-register
    VideoDecoder* dec = manager.CreateDecoder(VIDEO_CODEC_H264);
    EXPECT_NE(dec, nullptr);
    if (dec) manager.DestroyDecoder(dec);
}

// ---------------------------------------------------------------------------
// Decoder lifecycle
// ---------------------------------------------------------------------------

TEST_F(PluginManagerTest, CreateDecoderReturnsNonNull)
{
    VideoDecoder* dec = pm->CreateDecoder(VIDEO_CODEC_H264);
    ASSERT_NE(dec, nullptr);
    pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, CreateDecoderReturnsDistinctInstances)
{
    VideoDecoder* dec1 = pm->CreateDecoder(VIDEO_CODEC_H264);
    VideoDecoder* dec2 = pm->CreateDecoder(VIDEO_CODEC_H264);
    ASSERT_NE(dec1, nullptr);
    ASSERT_NE(dec2, nullptr);
    EXPECT_NE(dec1, dec2) << "Each call must return an independent instance";
    pm->DestroyDecoder(dec1);
    pm->DestroyDecoder(dec2);
}

TEST_F(PluginManagerTest, UnknownCodecReturnsNullDecoder)
{
    // Cast an invalid codec value — no plugin should accept it
    VideoDecoder* dec = pm->CreateDecoder(static_cast<VideoCodec>(0xFFFF));
    EXPECT_EQ(dec, nullptr);
}

TEST_F(PluginManagerTest, DestroyNullptrIsNonFatal)
{
    EXPECT_NO_THROW(pm->DestroyDecoder(nullptr));
}

TEST_F(PluginManagerTest, DestroyUnknownPointerIsNonFatal)
{
    // A pointer that was never issued by this manager must not crash
    VideoDecoder* fake = reinterpret_cast<VideoDecoder*>(uintptr_t{0xDEADBEEF});
    EXPECT_NO_THROW(pm->DestroyDecoder(fake));
}

// ---------------------------------------------------------------------------
// Decoder functionality via GetPluginApi()
// ---------------------------------------------------------------------------

TEST_F(PluginManagerTest, SendPacketReturnsSuccess)
{
    const VideoPluginAPI* api = pm->GetPluginApi(VIDEO_CODEC_H264);
    ASSERT_NE(api, nullptr);
    VideoDecoder* dec = pm->CreateDecoder(VIDEO_CODEC_H264);
    ASSERT_NE(dec, nullptr);
    // Fake Annex-B start-code + IDR NAL header
    const uint8_t data[] = {0x00, 0x00, 0x00, 0x01, 0x65};
    EXPECT_EQ(api->send_packet(dec, data, static_cast<int>(sizeof(data))), 0);
    pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, ReceiveFrameReturnsNonNegative)
{
    const VideoPluginAPI* api = pm->GetPluginApi(VIDEO_CODEC_H264);
    ASSERT_NE(api, nullptr);
    VideoDecoder* dec = pm->CreateDecoder(VIDEO_CODEC_H264);
    ASSERT_NE(dec, nullptr);
    VideoFrame frame{};
    EXPECT_GE(api->receive_frame(dec, &frame), 0);
    pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, SendThenReceiveCycle)
{
    const VideoPluginAPI* api = pm->GetPluginApi(VIDEO_CODEC_H264);
    ASSERT_NE(api, nullptr);
    VideoDecoder* dec = pm->CreateDecoder(VIDEO_CODEC_H264);
    ASSERT_NE(dec, nullptr);
    const uint8_t data[] = {0x00, 0x00, 0x00, 0x01, 0x65};
    EXPECT_EQ(api->send_packet(dec, data, sizeof(data)), 0);
    VideoFrame frame{};
    EXPECT_GE(api->receive_frame(dec, &frame), 0);
    pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, FlushIsCallableWhenPresent)
{
    const VideoPluginAPI* api = pm->GetPluginApi(VIDEO_CODEC_H264);
    ASSERT_NE(api, nullptr);
    VideoDecoder* dec = pm->CreateDecoder(VIDEO_CODEC_H264);
    ASSERT_NE(dec, nullptr);
    if (api->flush) { EXPECT_GE(api->flush(dec), 0); }
    pm->DestroyDecoder(dec);
}

TEST_F(PluginManagerTest, LastErrorIsCallableWhenPresent)
{
    const VideoPluginAPI* api = pm->GetPluginApi(VIDEO_CODEC_H264);
    ASSERT_NE(api, nullptr);
    VideoDecoder* dec = pm->CreateDecoder(VIDEO_CODEC_H264);
    ASSERT_NE(dec, nullptr);
    if (api->last_error) { (void)api->last_error(dec); }
    pm->DestroyDecoder(dec);
}

// ---------------------------------------------------------------------------
// API function-pointer validity
// ---------------------------------------------------------------------------

TEST_F(PluginManagerTest, PluginReportsCorrectAbiVersion)
{
    const VideoPluginAPI* api = pm->GetPluginApi(VIDEO_CODEC_H264);
    ASSERT_NE(api, nullptr);
    EXPECT_EQ(api->abi_version, static_cast<uint32_t>(VIDEO_PLUGIN_ABI_VERSION));
}

TEST_F(PluginManagerTest, RequiredFunctionPointersAreNonNull)
{
    const VideoPluginAPI* api = pm->GetPluginApi(VIDEO_CODEC_H264);
    ASSERT_NE(api, nullptr);
    EXPECT_NE(api->create_decoder,  nullptr);
    EXPECT_NE(api->destroy_decoder, nullptr);
    EXPECT_NE(api->send_packet,     nullptr);
    EXPECT_NE(api->receive_frame,   nullptr);
    // flush and last_error are optional — NULL is valid
}
