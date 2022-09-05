#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <ifm3d/device/o3x.h>
#include <gtest/gtest.h>

class O3XTest : public ::testing::Test
{
protected:
  virtual void
  SetUp()
  {
    this->dev_ =
      std::dynamic_pointer_cast<ifm3d::O3X>(ifm3d::Device::MakeShared());
  }

  virtual void
  TearDown()
  {}

  ifm3d::O3X::Ptr dev_;
};

TEST_F(O3XTest, AmI)
{
  EXPECT_NO_THROW(this->dev_->AmI(ifm3d::Device::device_family::O3X));
  bool isO3X = this->dev_->AmI(ifm3d::Device::device_family::O3X);
  EXPECT_EQ(isO3X, true);
}

TEST_F(O3XTest, ForceTrigger) { EXPECT_NO_THROW(dev_->ForceTrigger()); }

TEST_F(O3XTest, DeviceType)
{
  std::string device_type_from_device;
  EXPECT_NO_THROW(device_type_from_device = dev_->DeviceType(false));
  EXPECT_STREQ("768:1023", device_type_from_device.c_str());
}

TEST_F(O3XTest, WhoAmI)
{
  ifm3d::Device::device_family device;
  EXPECT_NO_THROW(device = dev_->WhoAmI());
  EXPECT_EQ(device, ifm3d::Device::device_family::O3X);
}