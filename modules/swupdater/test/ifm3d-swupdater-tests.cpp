#include <chrono>
#include <fstream>
#include <gtest/gtest.h>
#include <ifm3d/common/err.h>
#include <ifm3d/common/features.h>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/swupdater/swupdater.h>
#include <ios>
#include <memory>
#include <string>
#include <thread>

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

#ifdef BUILD_MODULE_CRYPTO
TEST_F(SWUpdater, DISABLED_DetectBootModePassword)
{
  auto cam = ifm3d::Device::MakeShared();
  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(cam);
  if (!o3r)
    {
      GTEST_SKIP() << "Device is not O3R";
    }

  EXPECT_FALSE(o3r->SealedBox()->IsPasswordProtected());
  const std::string password = "foo";
  o3r->SealedBox()->SetPassword(password);
  EXPECT_TRUE(o3r->SealedBox()->IsPasswordProtected());

  auto swu = std::make_shared<ifm3d::SWUpdater>(cam);

  EXPECT_THROW(swu->RebootToRecovery("wrong_password"), ifm3d::Error);
  EXPECT_NO_THROW(swu->RebootToRecovery(password));
  EXPECT_TRUE(swu->WaitForRecovery(100000));

  swu->RebootToProductive();
  EXPECT_TRUE(swu->WaitForProductive(100000));

  cam = ifm3d::Device::MakeShared();
  o3r = std::dynamic_pointer_cast<ifm3d::O3R>(cam);
  if (o3r && o3r->SealedBox()->IsPasswordProtected())
    {
      o3r->SealedBox()->RemovePassword(password);
    }
  EXPECT_FALSE(o3r->SealedBox()->IsPasswordProtected());
}
#endif

TEST_F(SWUpdater, DISABLED_FlashEmptyFile)
{
  auto cam = ifm3d::Device::MakeShared();
  auto swu = std::make_shared<ifm3d::SWUpdater>(cam);

  if (!cam->AmI(ifm3d::Device::DeviceFamily::O3R))
    {
      EXPECT_TRUE(swu->WaitForProductive(-1));
      EXPECT_FALSE(swu->WaitForRecovery(-1));
    }

  swu->RebootToRecovery();
  EXPECT_TRUE(swu->WaitForRecovery(80000));

  std::string const swu_file("swu_test_file.swu");
  std::fstream infile;
  infile.open(swu_file, std::ios::out);
  infile.close();

  EXPECT_THROW(swu->FlashFirmware(swu_file, 120000), ifm3d::Error);

  swu->RebootToProductive();
  EXPECT_TRUE(swu->WaitForProductive(80000));
}
