
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
#include <ifm3d/device/err.h>
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

  fg_->Start({ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ});

  auto frame = fg_->WaitForFrame().get();

  auto amplitude = frame->GetBuffer(ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE);
  auto distance = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
  auto xyz = frame->GetBuffer(ifm3d::buffer_id::XYZ);

  EXPECT_TRUE(amplitude.width() != 0);
  EXPECT_TRUE(distance.width() != 0);
  EXPECT_TRUE(xyz.nchannels() == 3);
}

TEST_F(FrameGrabberTest, BlankSchema3D)
{
  LOG(INFO) << "BlankSchema3D test";

  fg_->Start({});

  auto frame = fg_->WaitForFrame().get();

  auto amplitude = frame->GetBuffer(ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE);
  auto distance = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
  auto xyz = frame->GetBuffer(ifm3d::buffer_id::XYZ);

  EXPECT_TRUE(amplitude.width() != 0);
  EXPECT_TRUE(distance.width() != 0);
  EXPECT_TRUE(xyz.nchannels() == 3);
}

TEST_F(FrameGrabberTest, BlankSchema2D)
{
  LOG(INFO) << "BlankSchema2D test";

  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->dev_);

  auto config = o3r->Get();
  config["ports"]["port0"]["state"] = "RUN";
  o3r->Set(config);
  fg_ = std::make_shared<ifm3d::FrameGrabber>(dev_, 50010);

  fg_->Start({});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::JPEG_IMAGE));
  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::O3R_RGB_IMAGE_INFO));
}

TEST_F(FrameGrabberTest, schema_o3r_rgb_image_info)
{
  LOG(INFO) << "schema_o3r_rgb_image_info test";

  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->dev_);

  auto config = o3r->Get();
  config["ports"]["port0"]["state"] = "RUN";
  o3r->Set(config);
  fg_ = std::make_shared<ifm3d::FrameGrabber>(dev_, 50010);

  fg_->Start({ifm3d::buffer_id::O3R_RGB_IMAGE_INFO});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::O3R_RGB_IMAGE_INFO));
}

TEST_F(FrameGrabberTest, schema_o3r_dist_image_info)
{
  LOG(INFO) << "schema_o3r_dist_image_info test";

  fg_->Start({ifm3d::buffer_id::O3R_DISTANCE_IMAGE_INFO});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_NO_THROW(auto o3r_dist_image_info = frame->GetBuffer(
                    ifm3d::buffer_id::O3R_DISTANCE_IMAGE_INFO));
}

TEST_F(FrameGrabberTest, BufferIDException)
{
  LOG(INFO) << "BufferIDException test";

  fg_->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE));
  EXPECT_THROW(frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE),
               ifm3d::Error);
}

TEST_F(FrameGrabberTest, DistanceNoiseImage)
{
  LOG(INFO) << " distance noise image schema test";

  fg_->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ});

  auto frame = fg_->WaitForFrame().get();

  // as not part of schema
  EXPECT_ANY_THROW(frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE));
}

TEST_F(FrameGrabberTest, DistanceNoiseImage_type)
{
  LOG(INFO) << " distance noise image test";

  fg_->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_NOISE});

  auto frame = fg_->WaitForFrame().get();

  auto distance_noise_image =
    frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE);

  EXPECT_EQ(distance_noise_image.dataFormat(),
            ifm3d::pixel_format::FORMAT_32F);
}

TEST_F(FrameGrabberTest, onError)
{
  LOG(INFO) << " onError ";

  auto result = 0;
  auto frame_count = 0;

  fg_->OnError([&](const ifm3d::Error& err) { result++; });

  fg_->OnNewFrame([&](const ifm3d::Frame::Ptr& frame) { frame_count++; });

  fg_->Start({ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
              ifm3d::buffer_id::CONFIDENCE_IMAGE});

  std::this_thread::sleep_for(std::chrono::seconds(10));

  // this rebbot will cause network interruption and onError must get error
  dev_->Reboot();

  std::this_thread::sleep_for(std::chrono::seconds(10));

  // frame_count > 0 shows grabbing data successful
  // result > 0 error occurred and call back was called
  EXPECT_TRUE(frame_count > 0 && result > 0);

  // give enough time for device to completely reboot
  std::this_thread::sleep_for(std::chrono::seconds(90));
}

TEST_F(FrameGrabberTest, confidence_image_3D)
{
  LOG(INFO) << " confidence image test on 3D  ";

  fg_->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ,
              ifm3d::buffer_id::CONFIDENCE_IMAGE});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE));

  auto confidence_image = frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE);

  EXPECT_EQ(confidence_image.dataFormat(), ifm3d::pixel_format::FORMAT_16U);
}

TEST_F(FrameGrabberTest, confidence_image_2D)
{
  LOG(INFO) << " confidence image test on 2D  ";

  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->dev_);

  auto config = o3r->Get();
  config["ports"]["port0"]["state"] = "RUN";
  o3r->Set(config);
  fg_ = std::make_shared<ifm3d::FrameGrabber>(dev_, 50010);

  fg_->Start(
    {ifm3d::buffer_id::JPEG_IMAGE, ifm3d::buffer_id::CONFIDENCE_IMAGE});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_THROW(frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE),
               ifm3d::Error);
}
