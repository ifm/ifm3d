#include "ifm3d/common/err.h"
#include "ifm3d/device/legacy_device.h"
#include "ifm3d/device/device.h"
#include "ifm3d/pcicclient/pcicclient.h"
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include "gtest/gtest.h"

class PCICClientTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    this->dev = ifm3d::LegacyDevice::MakeShared();
  }

  void
  TearDown() override
  {}

  // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
  ifm3d::LegacyDevice::Ptr dev;
};

TEST_F(PCICClientTest, IncomingResponseMessage)
{
  //
  // PCICClientTest is not supported for O3X
  // so this test does not apply
  //
  if (dev->AmI(ifm3d::Device::device_family::O3X))
    {
      EXPECT_THROW(std::make_shared<ifm3d::PCICClient>(dev), ifm3d::Error);
      return;
    }

  ifm3d::PCICClient::Ptr const pc = std::make_shared<ifm3d::PCICClient>(dev);

  for (int i = 0; i < 5; ++i)
    {
      std::string const result = pc->Call("C?");
      EXPECT_GT(result.size(), 0);
    }

  pc->Stop();
}

TEST_F(PCICClientTest, InvalidCommandLength)
{
  //
  // PCICClientTest is not supported for O3X
  // so this test does not apply
  //
  if (dev->AmI(ifm3d::Device::device_family::O3X))
    {
      EXPECT_THROW(std::make_shared<ifm3d::PCICClient>(dev), ifm3d::Error);
      return;
    }

  ifm3d::PCICClient::Ptr const pc = std::make_shared<ifm3d::PCICClient>(dev);

  for (int i = 0; i < 5; ++i)
    {
      std::string const result = pc->Call("Ca?");
      EXPECT_STREQ(result.c_str(), "?");
    }

  pc->Stop();
}

TEST_F(PCICClientTest, PCICTimeout)
{
  std::string result;
  //
  // PCICClientTest is not supported for O3X
  // so this test does not apply
  //
  if (dev->AmI(ifm3d::Device::device_family::O3X))
    {
      EXPECT_THROW(std::make_shared<ifm3d::PCICClient>(dev), ifm3d::Error);
      return;
    }

  ifm3d::PCICClient::Ptr pc = std::make_shared<ifm3d::PCICClient>(dev);

  std::unique_ptr<std::thread> reboot_thread =
    std::make_unique<std::thread>([&] {
      EXPECT_NO_THROW(
        this->dev->Reboot(ifm3d::LegacyDevice::boot_mode::PRODUCTIVE));

      std::this_thread::sleep_for(std::chrono::seconds(5));
    });

  if (reboot_thread && reboot_thread->joinable())
    {
      reboot_thread->join();
    }

  for (int i = 0; i < 20; ++i)
    {
      result.clear();

      if (!pc->Call("V?", result, 5000))
        {
          pc = std::make_shared<ifm3d::PCICClient>(dev);
        }
    }

  EXPECT_GT(result.size(), 0);
  pc->Stop();
}
