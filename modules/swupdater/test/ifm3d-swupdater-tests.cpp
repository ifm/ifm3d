#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <ifm3d/swupdater.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/err.h>
#include <ifm3d/common/logging/log.h>
#include <gtest/gtest.h>
#include <fstream>

class SWUpdater : public ::testing::Test
{
protected:
  SWUpdater() = default;

  void
  TearDown() override
  {
    // Give the camera some extra time to 'settle' before moving to the next
    // test. In pratice, the camera needs a little bit of settle time after
    // presenting as 'productive' before certain things work (e.g. SW Trig).
    // This isn't really a concern in real SWUpdate scenarios, but it can cause
    // some failures in our unit tests depending on order of execution.
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
};

TEST_F(SWUpdater, FactoryDefaults)
{
  LOG_INFO("FactoryDefaults test");
  auto cam = ifm3d::LegacyDevice::MakeShared();

  EXPECT_NO_THROW(cam->FactoryReset());
  std::this_thread::sleep_for(std::chrono::seconds(6));
  EXPECT_NO_THROW(cam->DeviceType());
}

TEST_F(SWUpdater, DetectBootMode)
{
  auto cam = ifm3d::Device::MakeShared();
  auto swu = std::make_shared<ifm3d::SWUpdater>(cam);

  EXPECT_TRUE(swu->WaitForProductive(-1));
  EXPECT_FALSE(swu->WaitForRecovery(-1));
  EXPECT_FALSE(swu->WaitForRecovery(5000));

  swu->RebootToRecovery();
  EXPECT_TRUE(swu->WaitForRecovery(100000));

  EXPECT_FALSE(swu->WaitForProductive(-1));
  EXPECT_TRUE(swu->WaitForRecovery(-1));
  EXPECT_FALSE(swu->WaitForProductive(5000));

  swu->RebootToProductive();
  EXPECT_TRUE(swu->WaitForProductive(100000));

  EXPECT_TRUE(swu->WaitForProductive(-1));
  EXPECT_FALSE(swu->WaitForRecovery(-1));
}

TEST_F(SWUpdater, DISABLED_FlashEmptyFile)
{
  auto cam = ifm3d::Device::MakeShared();
  auto swu = std::make_shared<ifm3d::SWUpdater>(cam);

  if (!cam->AmI(ifm3d::Device::device_family::O3R))
    {
      EXPECT_TRUE(swu->WaitForProductive(-1));
      EXPECT_FALSE(swu->WaitForRecovery(-1));
    }

  swu->RebootToRecovery();
  EXPECT_TRUE(swu->WaitForRecovery(80000));

  std::string swu_file("swu_test_file.swu");
  std::fstream infile;
  infile.open(swu_file, std::ios::out);
  infile.close();

  EXPECT_THROW(swu->FlashFirmware(swu_file, 120000), ifm3d::Error);

  swu->RebootToProductive();
  EXPECT_TRUE(swu->WaitForProductive(80000));
}
