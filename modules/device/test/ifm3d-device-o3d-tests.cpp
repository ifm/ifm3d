#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <ifm3d/device/o3d.h>
#include <gtest/gtest.h>

class O3DTest : public ::testing::Test
{
protected:
  virtual void
  SetUp()
  {
    this->dev_ =
      std::dynamic_pointer_cast<ifm3d::O3D>(ifm3d::Device::MakeShared());
  }

  virtual void
  TearDown()
  {}

  ifm3d::O3D::Ptr dev_;
};

TEST_F(O3DTest, AmI)
{
  EXPECT_NO_THROW(this->dev_->AmI(ifm3d::Device::device_family::O3D));
  bool isO3D = this->dev_->AmI(ifm3d::Device::device_family::O3D);
  EXPECT_EQ(isO3D, true);
}

TEST_F(O3DTest, WhoAmI)
{
  ifm3d::Device::device_family device;
  EXPECT_NO_THROW(device = dev_->WhoAmI());
  EXPECT_EQ(device, ifm3d::Device::device_family::O3D);
}