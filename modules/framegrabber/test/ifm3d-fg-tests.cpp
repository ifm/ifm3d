
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <ifm3d/fg.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/o3r.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <iostream>

uint16_t o3r_port = 50012;

class FrameGrabberTest : public ::testing::Test
{
protected:
  virtual void
  SetUp()
  {
    this->dev_ = ifm3d::Device::MakeShared();
    if (dev_->WhoAmI() == ifm3d::Device::device_family::O3R)
      {
        auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->dev_);

        auto config = o3r->Get();
        config["ports"]["port2"]["state"] = "RUN";
        o3r->Set(config);
        fg_ = std::make_shared<ifm3d::FrameGrabber>(dev_, o3r_port);
      }
    else
      {
        fg_ = std::make_shared<ifm3d::FrameGrabber>(dev_);
      }
  }

  virtual void
  TearDown()
  {}

  ifm3d::Device::Ptr dev_;
  ifm3d::FrameGrabber::Ptr fg_;
};

TEST_F(FrameGrabberTest, WaitForFrame)
{
  LOG(INFO) << "WaitForFrame test";
  this->fg_->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
                    ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
                    ifm3d::buffer_id::XYZ});
  int i = 0;
  while (i < 10)
    {
      EXPECT_NO_THROW(this->fg_->WaitForFrame().get());
      i++;
    }

  EXPECT_EQ(i, 10);
}

TEST_F(FrameGrabberTest, CustomSchema)
{
  LOG(INFO) << "CustomSchema test";

  fg_->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ});

  auto frame = fg_->WaitForFrame().get();

  auto amplitude = frame->GetBuffer(ifm3d::buffer_id::AMPLITUDE_IMAGE);
  auto distance = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
  auto xyz = frame->GetBuffer(ifm3d::buffer_id::XYZ);

  EXPECT_TRUE(amplitude.width() != 0);
  EXPECT_TRUE(distance.width() != 0);
  EXPECT_TRUE(xyz.nchannels() == 3);
}
