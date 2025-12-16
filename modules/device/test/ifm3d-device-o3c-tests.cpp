#include <cstdlib>
#include <gtest/gtest.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3c.h>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

class O3CTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    this->_dev =
      std::dynamic_pointer_cast<ifm3d::O3C>(ifm3d::Device::MakeShared());
  }

  void
  TearDown() override
  {}

  ifm3d::O3C::Ptr _dev;
};

TEST_F(O3CTest, AmI)
{
  EXPECT_NO_THROW(this->_dev->AmI(ifm3d::Device::DeviceFamily::O3C));
  bool const is_o3c = this->_dev->AmI(ifm3d::Device::DeviceFamily::O3C);
  EXPECT_EQ(is_o3c, true);
  if (!is_o3c)
    {
      std::cout << "Device Mismatched, device under test is not O3C; "
                << "env. variable IFM3D_DEVICE_UNDR_TEST is =  "
                << std::getenv("IFM3D_DEVICE_UNDER_TEST") << '\n';
    }
}

TEST_F(O3CTest, WhoAmI_O3C)
{
  ifm3d::Device::DeviceFamily device{};
  EXPECT_NO_THROW(device = _dev->WhoAmI());
  EXPECT_EQ(device, ifm3d::Device::DeviceFamily::O3C);
}

TEST_F(O3CTest, SwUpdateVersion_O3C)
{
  ifm3d::Device::SWUVersion swu_version{};
  EXPECT_NO_THROW(swu_version = _dev->SwUpdateVersion());
  EXPECT_EQ(swu_version, ifm3d::Device::SWUVersion::SWU_V2);
}
TEST_F(O3CTest, FactoryReset)
{
  std::cout << "Testing FactoryReset keeping network settings\n";
  EXPECT_NO_THROW(this->_dev->FactoryReset(true));
}
TEST_F(O3CTest, GetSchema)
{
  EXPECT_NO_THROW({
    auto result = _dev->GetSchema();
    EXPECT_FALSE(result.empty());
  });

  EXPECT_NO_THROW({
    auto result = _dev->GetSchema("/ports");
    EXPECT_FALSE(result.empty());
  });

  std::string pointer = "/ports";
  EXPECT_NO_THROW({
    auto result = _dev->GetSchema(pointer);
    EXPECT_FALSE(result.empty());
  });

  EXPECT_NO_THROW({
    auto result = _dev->GetSchema({"/ports", "/applications"});
    EXPECT_FALSE(result.empty());
  });

  std::vector<std::string> pointer_vec = {"/ports", "/applications"};
  EXPECT_NO_THROW({
    auto result = _dev->GetSchema(pointer_vec);
    EXPECT_FALSE(result.empty());
  });
}