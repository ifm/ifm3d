#include "ifm3d/common/err.h"
#include "ifm3d/common/json_impl.hpp"
#include "ifm3d/device/device.h"
#include <functional>
#include <iostream>
#include <cstdlib>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>
#include <ifm3d/device/o3r.h>
#include <gtest/gtest.h>

class O3RTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    this->dev =
      std::dynamic_pointer_cast<ifm3d::O3R>(ifm3d::Device::MakeShared());
  }

  void
  TearDown() override
  {}

  // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
  ifm3d::O3R::Ptr dev;
};

TEST_F(O3RTest, AmI)
{
  EXPECT_NO_THROW(this->dev->AmI(ifm3d::Device::device_family::O3R));
  bool const is_O3R = this->dev->AmI(ifm3d::Device::device_family::O3R);
  EXPECT_EQ(is_O3R, true);
  if (!is_O3R)
    {
      std::cout << "Device Mismatched, device under test is not O3R; "
                << "env. variable IFM3D_DEVICE_UNDR_TEST is =  "
                << std::getenv("IFM3D_DEVICE_UNDER_TEST") << '\n';
    }
}

TEST_F(O3RTest, WhoAmI_O3R)
{
  ifm3d::Device::device_family device{};
  EXPECT_NO_THROW(device = dev->WhoAmI());
  EXPECT_EQ(device, ifm3d::Device::device_family::O3R);
}

TEST_F(O3RTest, portinfo)
{
  EXPECT_NO_THROW(dev->Ports());
  auto ports = dev->Ports();

  // atleast ports data will be available
  EXPECT_TRUE(!ports.empty());
}

TEST_F(O3RTest, port)
{
  EXPECT_NO_THROW(dev->Port("port0"));
  EXPECT_THROW(dev->Port("port7"), ifm3d::Error);
  EXPECT_THROW(dev->Port("random"), ifm3d::Error);
}

#ifdef BUILD_MODULE_CRYPTO

class RevertGuard
{
public:
  explicit RevertGuard(std::function<void()> fn)
    : _fn(std::move(std::move(fn)))
  {}

  RevertGuard(const RevertGuard&) = default;
  RevertGuard& operator=(const RevertGuard&) = default;

  RevertGuard(RevertGuard&&) = default;
  RevertGuard& operator=(RevertGuard&&) = default;

  ~RevertGuard()
  {
    try
      {
        _fn();
      }
    catch (...)
      {
        // Ignore any exceptions thrown during the revert process
      }
  }

private:
  std::function<void()> _fn;
};

TEST_F(O3RTest, SealedBox_IsPasswordProtected)
{
  EXPECT_NO_THROW(dev->SealedBox()->IsPasswordProtected());
}

TEST_F(O3RTest, SealedBox_GetPublicKey)
{
  EXPECT_NO_THROW(dev->SealedBox()->GetPublicKey());
}

TEST_F(O3RTest, SealedBox_SetPassword)
{
  EXPECT_FALSE(dev->SealedBox()->IsPasswordProtected())
    << "Device is already password protected, make sure device is not "
       "password protected before running the tests";

  auto revert_password =
    RevertGuard([this]() { dev->SealedBox()->RemovePassword("foo"); });
  EXPECT_NO_THROW(dev->SealedBox()->SetPassword("foo"));
}

TEST_F(O3RTest, SealedBox_ChangePassword)
{
  EXPECT_FALSE(dev->SealedBox()->IsPasswordProtected())
    << "Device is already password protected, make sure device is not "
       "password protected before running the tests";

  auto revert_password =
    RevertGuard([this]() { dev->SealedBox()->RemovePassword("foo"); });
  EXPECT_NO_THROW(dev->SealedBox()->SetPassword("foo"));

  auto revert_password2 =
    RevertGuard([this]() { dev->SealedBox()->RemovePassword("bar"); });
  EXPECT_NO_THROW(dev->SealedBox()->SetPassword("bar", "foo"));
}

TEST_F(O3RTest, SealedBox_SetConfig)
{
  EXPECT_FALSE(dev->SealedBox()->IsPasswordProtected())
    << "Device is already password protected, make sure device is not "
       "password protected before running the tests";

  auto authorized_keys = dev->ResolveConfig(
    ifm3d::json::json_pointer("/device/network/authorized_keys"));
  auto revert_authorized_keys = RevertGuard([this, authorized_keys]() {
    dev->Set(
      {{"device", {{"network", {{"authorized_keys", authorized_keys}}}}}});
  });

  auto revert_password =
    RevertGuard([this]() { dev->SealedBox()->RemovePassword("foo"); });
  EXPECT_NO_THROW(dev->SealedBox()->SetPassword("foo"));

  EXPECT_NO_THROW(dev->SealedBox()->Set(
    "foo",
    {{"device", {{"network", {{"authorized_keys", "baz"}}}}}}));

  EXPECT_EQ(dev->ResolveConfig(
              ifm3d::json::json_pointer("/device/network/authorized_keys")),
            "baz");
}

#endif