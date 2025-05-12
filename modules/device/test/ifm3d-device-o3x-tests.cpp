#include "ifm3d/device/device.h"
#include <memory>
#include <ifm3d/device/o3x.h>
#include <gtest/gtest.h>

class O3XTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    this->dev =
      std::dynamic_pointer_cast<ifm3d::O3X>(ifm3d::Device::MakeShared());
  }

  void
  TearDown() override
  {}

  // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
  ifm3d::O3X::Ptr dev;
};

TEST_F(O3XTest, AmI)
{
  EXPECT_NO_THROW(this->dev->AmI(ifm3d::Device::device_family::O3X));
  bool const is_O3X = this->dev->AmI(ifm3d::Device::device_family::O3X);
  EXPECT_EQ(is_O3X, true);
}

TEST_F(O3XTest, ForceTrigger) { EXPECT_NO_THROW(dev->ForceTrigger()); }

TEST_F(O3XTest, WhoAmI)
{
  ifm3d::Device::device_family device{};
  EXPECT_NO_THROW(device = dev->WhoAmI());
  EXPECT_EQ(device, ifm3d::Device::device_family::O3X);
}