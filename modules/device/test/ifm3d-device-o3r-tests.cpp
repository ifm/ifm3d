#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <ifm3d/device/o3r.h>
#include <ifm3d/device/err.h>
#include <gtest/gtest.h>

class O3RTest : public ::testing::Test
{
protected:
  virtual void
  SetUp()
  {
    this->dev_ =
      std::dynamic_pointer_cast<ifm3d::O3R>(ifm3d::Device::MakeShared());
  }

  virtual void
  TearDown()
  {}

  ifm3d::O3R::Ptr dev_;
};

TEST_F(O3RTest, AmI)
{
  EXPECT_NO_THROW(this->dev_->AmI(ifm3d::Device::device_family::O3R));
  bool isO3R = this->dev_->AmI(ifm3d::Device::device_family::O3R);
  EXPECT_EQ(isO3R, true);
  if (!isO3R)
    {
      std::cout << "Device Mismatched, device under test is not O3R; "
                << "env. variable IFM3D_DEVICE_UNDR_TEST is =  "
                << std::getenv("IFM3D_DEVICE_UNDER_TEST") << std::endl;
    }
}

TEST_F(O3RTest, WhoAmI_O3R)
{
  ifm3d::Device::device_family device;
  EXPECT_NO_THROW(device = dev_->WhoAmI());
  EXPECT_EQ(device, ifm3d::Device::device_family::O3R);
}

TEST_F(O3RTest, portinfo)
{
  EXPECT_NO_THROW(dev_->Ports());
  auto ports = dev_->Ports();

  // atleast ports data will be available
  EXPECT_TRUE(ports.size() > 0);
}

TEST_F(O3RTest, port)
{
  EXPECT_NO_THROW(dev_->Port("port0"));
  EXPECT_THROW(dev_->Port("port7"), ifm3d::Error);
  EXPECT_THROW(dev_->Port("random"), ifm3d::Error);
}