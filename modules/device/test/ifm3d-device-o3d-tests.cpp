#include "ifm3d/device/device.h"
#include <memory>
#include <ifm3d/device/o3d.h>
#include <gtest/gtest.h>

class O3DTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    this->dev =
      std::dynamic_pointer_cast<ifm3d::O3D>(ifm3d::Device::MakeShared());
  }

  void
  TearDown() override
  {}

  // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
  ifm3d::O3D::Ptr dev;
};

TEST_F(O3DTest, AmI)
{
  EXPECT_NO_THROW(this->dev->AmI(ifm3d::Device::device_family::O3D));
  bool const is_O3D = this->dev->AmI(ifm3d::Device::device_family::O3D);
  EXPECT_EQ(is_O3D, true);
}

TEST_F(O3DTest, WhoAmI)
{
  ifm3d::Device::device_family device{};
  EXPECT_NO_THROW(device = dev->WhoAmI());
  EXPECT_EQ(device, ifm3d::Device::device_family::O3D);
}