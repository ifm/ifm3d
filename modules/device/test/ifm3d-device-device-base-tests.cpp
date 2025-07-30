#include <chrono>
#include <string>
#include <thread>
#include <ifm3d/device/device.h>
#include <gtest/gtest.h>

class DeviceTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    this->dev = ifm3d::Device::MakeShared();
  }

  void
  TearDown() override
  {}

  // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
  ifm3d::Device::Ptr dev;
};

TEST_F(DeviceTest, DefaultCredentials)
{
  EXPECT_STREQ(this->dev->IP().c_str(), ifm3d::DEFAULT_IP.c_str());
  EXPECT_EQ(this->dev->XMLRPCPort(), ifm3d::DEFAULT_XMLRPC_PORT);
}

TEST_F(DeviceTest, DeviceDiscovery)
{
  EXPECT_NO_THROW(ifm3d::Device::DeviceDiscovery());
}

TEST_F(DeviceTest, DISABLED_Reboot_productive)
{
  EXPECT_NO_THROW(dev->Reboot());
  std::this_thread::sleep_for(std::chrono::seconds(60));
  EXPECT_NO_THROW(ifm3d::Device::MakeShared());
}

TEST_F(DeviceTest, DeviceType)
{
  std::string device_type_from_cache;
  EXPECT_NO_THROW(device_type_from_cache = dev->DeviceType());
  std::string device_type_from_device;
  EXPECT_NO_THROW(device_type_from_device = dev->DeviceType(false));
  EXPECT_STREQ(device_type_from_cache.c_str(),
               device_type_from_device.c_str());
}

TEST_F(DeviceTest, WhoAmI) { EXPECT_NO_THROW(dev->WhoAmI()); }