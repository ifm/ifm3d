#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <ifm3d/swupdater.h>
#include <ifm3d/camera/camera.h>
#include <ifm3d/camera/err.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

class SWUpdater : public ::testing::Test
{
protected:
  SWUpdater() = default;

  void TearDown() override
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
  LOG(INFO) << "FactoryDefaults test";
  auto cam = ifm3d::Camera::MakeShared();

  EXPECT_NO_THROW(cam->FactoryReset());
  std::this_thread::sleep_for(std::chrono::seconds(6));
  EXPECT_NO_THROW(cam->DeviceType());
}

TEST_F(SWUpdater, DetectBootMode)
{
  auto cam = ifm3d::Camera::MakeShared();
  auto swu = std::make_shared<ifm3d::SWUpdater>(cam);

  EXPECT_TRUE(swu->WaitForProductive(-1));
  EXPECT_FALSE(swu->WaitForRecovery(-1));
  EXPECT_FALSE(swu->WaitForRecovery(5000));

  swu->RebootToRecovery();
  EXPECT_TRUE(swu->WaitForRecovery(60000));

  EXPECT_FALSE(swu->WaitForProductive(-1));
  EXPECT_TRUE(swu->WaitForRecovery(-1));
  EXPECT_FALSE(swu->WaitForProductive(5000));

  swu->RebootToProductive();
  EXPECT_TRUE(swu->WaitForProductive(60000));

  EXPECT_TRUE(swu->WaitForProductive(-1));
  EXPECT_FALSE(swu->WaitForRecovery(-1));
}

TEST_F(SWUpdater, FlashEmptyFile)
{
  auto cam = ifm3d::Camera::MakeShared();
  auto swu = std::make_shared<ifm3d::SWUpdater>(cam);

  EXPECT_TRUE(swu->WaitForProductive(-1));
  EXPECT_FALSE(swu->WaitForRecovery(-1));

  swu->RebootToRecovery();
  EXPECT_TRUE(swu->WaitForRecovery(60000));

  std::vector<std::uint8_t> bytes(100000, 0);
  EXPECT_THROW(swu->FlashFirmware(bytes, 120000), ifm3d::error_t);

  swu->RebootToProductive();
  EXPECT_TRUE(swu->WaitForProductive(60000));
}

