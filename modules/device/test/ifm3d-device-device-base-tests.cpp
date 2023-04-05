#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <ifm3d/device/device.h>
#include <gtest/gtest.h>

class DeviceTest : public ::testing::Test
{
protected:
  virtual void
  SetUp()
  {
    this->dev_ = ifm3d::Device::MakeShared();
  }

  virtual void
  TearDown()
  {}

  ifm3d::Device::Ptr dev_;
};

TEST_F(DeviceTest, DefaultCredentials)
{
  EXPECT_STREQ(this->dev_->IP().c_str(), ifm3d::DEFAULT_IP.c_str());
  EXPECT_EQ(this->dev_->XMLRPCPort(), ifm3d::DEFAULT_XMLRPC_PORT);
}

TEST_F(DeviceTest, DeviceDiscovery)
{
  EXPECT_NO_THROW(ifm3d::Device::DeviceDiscovery());
}

TEST_F(DeviceTest, DISABLED_Reboot_productive)
{
  EXPECT_NO_THROW(dev_->Reboot());
  std::this_thread::sleep_for(std::chrono::seconds(60));
  EXPECT_NO_THROW(ifm3d::Device::MakeShared());
}

TEST_F(DeviceTest, DeviceType)
{
  std::string device_type_from_cache;
  EXPECT_NO_THROW(device_type_from_cache = dev_->DeviceType());
  std::string device_type_from_device;
  EXPECT_NO_THROW(device_type_from_device = dev_->DeviceType(false));
  EXPECT_STREQ(device_type_from_cache.c_str(),
               device_type_from_device.c_str());
}

TEST_F(DeviceTest, WhoAmI) { EXPECT_NO_THROW(dev_->WhoAmI()); }