#include <gtest/gtest.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3d.h>
#include <memory>

class O3DTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    this->_dev =
      std::dynamic_pointer_cast<ifm3d::O3D>(ifm3d::Device::MakeShared());
  }

  void
  TearDown() override
  {}

  // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
  ifm3d::O3D::Ptr _dev;
};

TEST_F(O3DTest, AmI)
{
  EXPECT_NO_THROW(this->_dev->AmI(ifm3d::Device::device_family::O3D));
  bool const is_o3d = this->_dev->AmI(ifm3d::Device::device_family::O3D);
  EXPECT_EQ(is_o3d, true);
}

TEST_F(O3DTest, WhoAmI)
{
  ifm3d::Device::device_family device{};
  EXPECT_NO_THROW(device = _dev->WhoAmI());
  EXPECT_EQ(device, ifm3d::Device::device_family::O3D);
}