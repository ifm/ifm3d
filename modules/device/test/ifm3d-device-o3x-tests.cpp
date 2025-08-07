#include <gtest/gtest.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3x.h>
#include <memory>

class O3XTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    this->_dev =
      std::dynamic_pointer_cast<ifm3d::O3X>(ifm3d::Device::MakeShared());
  }

  void
  TearDown() override
  {}

  // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
  ifm3d::O3X::Ptr _dev;
};

TEST_F(O3XTest, AmI)
{
  EXPECT_NO_THROW(this->_dev->AmI(ifm3d::Device::device_family::O3X));
  bool const is_o3x = this->_dev->AmI(ifm3d::Device::device_family::O3X);
  EXPECT_EQ(is_o3x, true);
}

TEST_F(O3XTest, ForceTrigger) { EXPECT_NO_THROW(_dev->ForceTrigger()); }

TEST_F(O3XTest, WhoAmI)
{
  ifm3d::Device::device_family device{};
  EXPECT_NO_THROW(device = _dev->WhoAmI());
  EXPECT_EQ(device, ifm3d::Device::device_family::O3X);
}