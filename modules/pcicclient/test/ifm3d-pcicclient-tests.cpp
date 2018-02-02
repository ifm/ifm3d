#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include "gtest/gtest.h"
#include <ifm3d/camera.h>
#include <ifm3d/pcicclient.h>

class PCICClientTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    this->cam_ = ifm3d::Camera::MakeShared();
  }

  virtual void TearDown()
  {

  }

  ifm3d::Camera::Ptr cam_;
};

TEST_F(PCICClientTest, IncomingResponseMessage)
{
  //
  // PCICClientTest is not supported for O3X
  // so this test does not apply
  //
  if (cam_->IsO3X())
    {
      EXPECT_THROW(std::make_shared<ifm3d::PCICClient>(cam_), ifm3d::error_t);
      return;
    }

  ifm3d::PCICClient::Ptr pc = std::make_shared<ifm3d::PCICClient>(cam_);

  for(int i = 0; i < 5; ++i)
    {
      std::string result = pc->Call("C?");
      EXPECT_GT(result.size(), 0);
    }

  pc->Stop();
}

TEST_F(PCICClientTest, InvalidCommandLength)
{
  //
  // PCICClientTest is not supported for O3X
  // so this test does not apply
  //
  if (cam_->IsO3X())
    {
      EXPECT_THROW(std::make_shared<ifm3d::PCICClient>(cam_), ifm3d::error_t);
      return;
    }

  ifm3d::PCICClient::Ptr pc = std::make_shared<ifm3d::PCICClient>(cam_);

  for(int i = 0; i < 5; ++i)
    {
      std::string result = pc->Call("Ca?");
      EXPECT_STREQ(result.c_str(), "?");
    }

  pc->Stop();
}
