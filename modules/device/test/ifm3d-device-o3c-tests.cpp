#include <cstdlib>
#include <exception>
#include <functional>
#include <gtest/gtest.h>
#include <ifm3d/common/err.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3c.h>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
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

TEST_F(O3CTest, portinfo)
{
  EXPECT_NO_THROW(_dev->Ports());
  auto ports = _dev->Ports();

  // atleast ports data will be available
  EXPECT_TRUE(!ports.empty());
}

TEST_F(O3CTest, port)
{
  EXPECT_NO_THROW(_dev->Port("port0"));
  EXPECT_NO_THROW(_dev->Port("port6"));
  EXPECT_THROW(_dev->Port("random"), ifm3d::Error);
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
    catch (const std::exception& e)
      {
        // Ignore any exceptions thrown during the revert process
      }
  }

private:
  std::function<void()> _fn;
};

TEST_F(O3CTest, SealedBox_IsPasswordProtected)
{
  EXPECT_NO_THROW(_dev->SealedBox()->IsPasswordProtected());
}

TEST_F(O3CTest, SealedBox_GetPublicKey)
{
  EXPECT_NO_THROW(_dev->SealedBox()->GetPublicKey());
}

TEST_F(O3CTest, SealedBox_SetPassword)
{
  EXPECT_FALSE(_dev->SealedBox()->IsPasswordProtected())
    << "Device is already password protected, make sure device is not "
       "password protected before running the tests";

  auto revert_password =
    RevertGuard([this]() { _dev->SealedBox()->RemovePassword("foo"); });
  EXPECT_NO_THROW(_dev->SealedBox()->SetPassword("foo"));
}

TEST_F(O3CTest, SealedBox_ChangePassword)
{
  EXPECT_FALSE(_dev->SealedBox()->IsPasswordProtected())
    << "Device is already password protected, make sure device is not "
       "password protected before running the tests";

  auto revert_password =
    RevertGuard([this]() { _dev->SealedBox()->RemovePassword("foo"); });
  EXPECT_NO_THROW(_dev->SealedBox()->SetPassword("foo"));

  auto revert_password2 =
    RevertGuard([this]() { _dev->SealedBox()->RemovePassword("bar"); });
  EXPECT_NO_THROW(_dev->SealedBox()->SetPassword("bar", "foo"));
}

TEST_F(O3CTest, SealedBox_SetConfig)
{
  EXPECT_FALSE(_dev->SealedBox()->IsPasswordProtected())
    << "Device is already password protected, make sure device is not "
       "password protected before running the tests";

  auto authorized_keys = _dev->ResolveConfig(
    ifm3d::json::json_pointer("/device/network/authorized_keys"));
  auto revert_authorized_keys = RevertGuard([this, authorized_keys]() {
    _dev->Set(
      {{"device", {{"network", {{"authorized_keys", authorized_keys}}}}}});
  });

  auto revert_password =
    RevertGuard([this]() { _dev->SealedBox()->RemovePassword("foo"); });
  EXPECT_NO_THROW(_dev->SealedBox()->SetPassword("foo"));

  EXPECT_NO_THROW(_dev->SealedBox()->Set(
    "foo",
    {{"device", {{"network", {{"authorized_keys", "baz"}}}}}}));

  EXPECT_EQ(_dev->ResolveConfig(
              ifm3d::json::json_pointer("/device/network/authorized_keys")),
            "baz");
}

#endif