#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include "gtest/gtest.h"
#include <ifm3d/device.h>
#include <ifm3d/pcicclient.h>

class PCICClientTest : public ::testing::Test
{
protected:
  virtual void
  SetUp()
  {
    this->dev_ = ifm3d::LegacyDevice::MakeShared();
  }

  virtual void
  TearDown()
  {}

  ifm3d::LegacyDevice::Ptr dev_;
};

TEST_F(PCICClientTest, IncomingResponseMessage)
{
  //
  // PCICClientTest is not supported for O3X
  // so this test does not apply
  //
  if (dev_->AmI(ifm3d::Device::device_family::O3X))
    {
      EXPECT_THROW(std::make_shared<ifm3d::PCICClient>(dev_), ifm3d::Error);
      return;
    }

  ifm3d::PCICClient::Ptr pc = std::make_shared<ifm3d::PCICClient>(dev_);

  for (int i = 0; i < 5; ++i)
    {
      std::string result = pc->Call("C?");
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
  if (dev_->AmI(ifm3d::Device::device_family::O3X))
    {
      EXPECT_THROW(std::make_shared<ifm3d::PCICClient>(dev_), ifm3d::Error);
      return;
    }

  ifm3d::PCICClient::Ptr pc = std::make_shared<ifm3d::PCICClient>(dev_);

  for (int i = 0; i < 5; ++i)
    {
      std::string result = pc->Call("Ca?");
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
  if (dev_->AmI(ifm3d::Device::device_family::O3X))
    {
      EXPECT_THROW(std::make_shared<ifm3d::PCICClient>(dev_), ifm3d::Error);
      return;
    }

  ifm3d::PCICClient::Ptr pc = std::make_shared<ifm3d::PCICClient>(dev_);

  std::unique_ptr<std::thread> reboot_thread_ =
    std::make_unique<std::thread>([&] {
      EXPECT_NO_THROW(
        this->dev_->Reboot(ifm3d::LegacyDevice::boot_mode::PRODUCTIVE));

      std::this_thread::sleep_for(std::chrono::seconds(5));
    });

  if (reboot_thread_ && reboot_thread_->joinable())
    reboot_thread_->join();

  for (int i = 0; i < 20; ++i)
    {
      result.clear();

      if (!pc->Call("V?", result, 5000))
        {
          pc = std::make_shared<ifm3d::PCICClient>(dev_);
        }
    }

  EXPECT_GT(result.size(), 0);
  pc->Stop();
}
