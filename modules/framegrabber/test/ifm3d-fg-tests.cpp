
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
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/o3d.h>
#include <ifm3d/device/o3x.h>

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
#if 0
TEST_F(FrameGrabberTest, start_stop_start)
{
  LOG_INFO("start_stop_start test");
  for (int itr = 0; itr < 10; itr++)
    {

      EXPECT_EQ(this->fg_->Start({}).wait_for(std::chrono::seconds(1)),
                std::future_status::ready);
      int i = 0;
      while (i < 10)
        {
          EXPECT_NO_THROW(this->fg_->WaitForFrame().get());
          i++;
        }

      EXPECT_EQ(i, 10);

      EXPECT_EQ(this->fg_->Stop().wait_for(std::chrono::seconds(1)),
                std::future_status::ready);

      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

TEST_F(FrameGrabberTest, masking)
{
  LOG_INFO("enabling disabling masking test");
  int frame_count = 0;
  this->fg_->OnNewFrame([&frame_count](auto frame) { frame_count++; });
  this->fg_->Start({});

  for (int itr = 0; itr < 10; itr++)
    {
      fg_->SetMasking(true);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      EXPECT_TRUE(fg_->IsMasking());
      std::this_thread::sleep_for(std::chrono::seconds(1));
      fg_->SetMasking(false);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      EXPECT_FALSE(fg_->IsMasking());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  this->fg_->Stop();
}
#endif

TEST_F(FrameGrabberTest, FactoryDefaults)
{
  EXPECT_NO_THROW(
    std::dynamic_pointer_cast<ifm3d::LegacyDevice>(dev_)->FactoryReset());
  std::this_thread::sleep_for(std::chrono::seconds(6));
  EXPECT_NO_THROW(this->dev_->DeviceType());
}

TEST_F(FrameGrabberTest, WaitForFrame)
{
  LOG_INFO("WaitForFrame test");
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
  LOG_INFO("CustomSchema test");

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
  LOG_INFO("BlankSchema3D test");

  fg_->Start({});

  auto frame = fg_->WaitForFrame().get();

  auto distance = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
  auto xyz = frame->GetBuffer(ifm3d::buffer_id::XYZ);

  EXPECT_TRUE(distance.width() != 0);
  EXPECT_TRUE(xyz.nchannels() == 3);
}

TEST_F(FrameGrabberTest, BlankSchema2D)
{
  LOG_INFO("BlankSchema2D test");

  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->dev_);

  auto config = o3r->Get();
  config["ports"]["port0"]["state"] = "RUN";
  o3r->Set(config);
  fg_ = std::make_shared<ifm3d::FrameGrabber>(dev_, 50010);

  fg_->Start({});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::JPEG_IMAGE));
  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::RGB_INFO));
}

TEST_F(FrameGrabberTest, schema_o3r_rgb_image_info)
{
  LOG_INFO("schema_o3r_rgb_image_info test");

  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->dev_);

  auto config = o3r->Get();
  config["ports"]["port0"]["state"] = "RUN";
  o3r->Set(config);
  fg_ = std::make_shared<ifm3d::FrameGrabber>(dev_, 50010);

  fg_->Start({ifm3d::buffer_id::RGB_INFO});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::RGB_INFO));
}

TEST_F(FrameGrabberTest, schema_o3r_dist_image_info)
{
  LOG_INFO("schema_o3r_dist_image_info test");

  fg_->Start({ifm3d::buffer_id::TOF_INFO});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_NO_THROW(auto o3r_dist_image_info =
                    frame->GetBuffer(ifm3d::buffer_id::TOF_INFO));
}

TEST_F(FrameGrabberTest, BufferIDException)
{
  LOG_INFO("BufferIDException test");

  fg_->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ});

  auto frame = fg_->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE));
  EXPECT_THROW(frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE),
               ifm3d::Error);
}

TEST_F(FrameGrabberTest, DistanceNoiseImageSchema)
{
  LOG_INFO(" distance noise image schema test");

  fg_->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ});

  auto frame = fg_->WaitForFrame().get();

  // as not part of schema
  EXPECT_ANY_THROW(frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE));
}

TEST_F(FrameGrabberTest, DistanceNoiseImage_type)
{
  LOG_INFO(" distance noise image test");

  fg_
    ->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
             ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
             ifm3d::buffer_id::RADIAL_DISTANCE_NOISE})
    .wait_for(std::chrono::seconds(2));
  size_t count = 0;
  fg_->OnNewFrame([&](ifm3d::Frame::Ptr frame) {
    if (frame->HasBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE))
      {
        count++;
        auto distance_noise_image =
          frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE);
        if (dev_->AmI(ifm3d::Device::device_family::O3R))
          {
            EXPECT_EQ(distance_noise_image.dataFormat(),
                      ifm3d::pixel_format::FORMAT_32F);
          }
        if (dev_->AmI(ifm3d::Device::device_family::O3X))
          {
            EXPECT_EQ(distance_noise_image.dataFormat(),
                      ifm3d::pixel_format::FORMAT_16U);
          }
        fg_->Stop();
      }
    else if (count == 10)
      {
        fg_->Stop();
        EXPECT_TRUE(false);
      }
  });
}

TEST_F(FrameGrabberTest, onError)
{
  LOG_INFO(" onError ");

  auto result = 0;
  auto frame_count = 0;

  fg_->OnError([&](const ifm3d::Error& err) { result++; });

  fg_->OnNewFrame([&](const ifm3d::Frame::Ptr& frame) { frame_count++; });

  fg_->Start({ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
              ifm3d::buffer_id::CONFIDENCE_IMAGE});

  std::this_thread::sleep_for(std::chrono::seconds(10));

  // this reboot will cause network interruption and onError must get error
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
  // LOG_INFO(" confidence image test on 3D  ");

  fg_
    ->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
             ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
             ifm3d::buffer_id::XYZ,
             ifm3d::buffer_id::CONFIDENCE_IMAGE})
    .wait_for(std::chrono::seconds(1));
  size_t count = 0;
  fg_->OnNewFrame([&](ifm3d::Frame::Ptr frame) {
    count++;
    if (frame->HasBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE))
      {

        EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE));

        auto confidence_image =
          frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE);

        if (dev_->AmI(ifm3d::Device::device_family::O3R))
          {
            EXPECT_EQ(confidence_image.dataFormat(),
                      ifm3d::pixel_format::FORMAT_16U);
          }
        else
          {
            EXPECT_EQ(confidence_image.dataFormat(),
                      ifm3d::pixel_format::FORMAT_8U);
          }
        fg_->Stop();
      }
    else if (count == 10)
      {
        fg_->Stop();
        EXPECT_TRUE(false);
      }
  });
}

TEST_F(FrameGrabberTest, confidence_image_2D)
{
  LOG_INFO(" confidence image test on 2D  ");

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

TEST_F(FrameGrabberTest, only_algo_debug)
{
  LOG_INFO(" obtain only algo debug data");
  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->dev_);

  // enable algo debug flag through xmlrpc set interface
  o3r->Reset("/ports/port2/data/algoDebugFlag");
  o3r->Set({{"ports", {{"port2", {{"data", {{"algoDebugFlag", true}}}}}}}});

  std::this_thread::sleep_for(std::chrono::seconds(1));

  fg_->Start({ifm3d::buffer_id::ALGO_DEBUG});

  auto future_ = fg_->WaitForFrame();
  auto status = future_.wait_for(std::chrono::seconds(1));
  EXPECT_TRUE(status == std::future_status::ready);

  auto frame = future_.get();
  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::ALGO_DEBUG));
  EXPECT_THROW(frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE),
               ifm3d::Error);
}

TEST_F(FrameGrabberTest, algo_with_other_data)
{
  LOG_INFO(" obtain  algo debug with other data");
  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->dev_);

  // enable algo debug flag through xmlrpc set interface
  o3r->Reset("/ports/port2/data/algoDebugFlag");
  o3r->Set({{"ports", {{"port2", {{"data", {{"algoDebugFlag", true}}}}}}}});

  std::this_thread::sleep_for(std::chrono::seconds(1));

  fg_->Start(
    {ifm3d::buffer_id::ALGO_DEBUG, ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE});

  for (int i = 0; i < 20; i++)
    {
      auto frame = fg_->WaitForFrame().get();

      if (frame->HasBuffer(ifm3d::buffer_id::ALGO_DEBUG))
        {
          EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::ALGO_DEBUG));
          EXPECT_THROW(
            frame->GetBuffer(ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE),
            ifm3d::Error);
        }
      else
        {
          EXPECT_THROW(frame->GetBuffer(ifm3d::buffer_id::ALGO_DEBUG),
                       ifm3d::Error);
          EXPECT_NO_THROW(
            frame->GetBuffer(ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE));
        }
    }
}

TEST_F(FrameGrabberTest, StartStopStart)
{
  bool has_error = false;
  fg_->OnError([&has_error](ifm3d::Error e) {
    LOG_ERROR(e.what());
    has_error = true;
  });

  for (int i = 0; i < 3; ++i)
    {
      EXPECT_FALSE(fg_->IsRunning());

      EXPECT_TRUE(fg_->Start({}).wait_for(std::chrono::seconds(5)) ==
                  std::future_status::ready);

      EXPECT_TRUE(fg_->WaitForFrame().wait_for(std::chrono::seconds(1)) ==
                  std::future_status::ready);

      EXPECT_TRUE(fg_->IsRunning());

      EXPECT_TRUE(fg_->Stop().wait_for(std::chrono::seconds(5)) ==
                  std::future_status::ready);

      EXPECT_FALSE(fg_->IsRunning());
    }

  EXPECT_FALSE(has_error);
}

TEST_F(FrameGrabberTest, FrameGrabberRecycling)
{
  LOG_INFO("FrameGrabberRecycling test");
  fg_->Start({});

  for (int i = 0; i < 5; ++i)
    {
      auto status =
        fg_->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
      EXPECT_TRUE(status == std::future_status::ready);
    }
  fg_.reset();
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
  fg_->Start({});
  for (int i = 0; i < 5; ++i)
    {
      auto status =
        fg_->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
      EXPECT_TRUE(status == std::future_status::ready);
    }
}

TEST_F(FrameGrabberTest, SoftwareTrigger)
{
  LOG_INFO("SoftwareTrigger test");

  auto legacy_device = std::dynamic_pointer_cast<ifm3d::LegacyDevice>(dev_);
  // mark the current active application as sw triggered
  int idx = legacy_device->ActiveApplication();
  ifm3d::json config = legacy_device->ToJSON();
  config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Device::trigger_mode::SW));
  legacy_device->FromJSON(config);

  fg_->Start({});

  // waiting for an image should now timeout
  auto status = fg_->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
  EXPECT_TRUE(status == std::future_status::timeout);

  // now, get image data by explicitly s/w triggering the device
  for (int i = 0; i < 10; ++i)
    {
      fg_->SWTrigger();
      auto status =
        fg_->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
      EXPECT_TRUE(status == std::future_status::ready);
    }

  // set the camera back into free-run mode
  config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Device::trigger_mode::FREE_RUN));
  dev_->FromJSON(config);
}

TEST_F(FrameGrabberTest, SWTriggerMultipleClients)
{
  LOG_INFO("SWTriggerMultipleClients test");

  auto legacy_device = std::dynamic_pointer_cast<ifm3d::LegacyDevice>(dev_);
  // mark the current active application as sw triggered
  int idx = legacy_device->ActiveApplication();
  ifm3d::json config = legacy_device->ToJSON();
  config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Device::trigger_mode::SW));
  legacy_device->FromJSON(config);

  // create two framegrabbers and two buffers
  auto fg1 = std::make_shared<ifm3d::FrameGrabber>(legacy_device, 50010);
  auto fg2 = fg_;

  fg1->Start({});
  fg2->Start({});

  // waiting for an image should now timeout
  auto status1 = fg1->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
  EXPECT_TRUE(status1 == std::future_status::timeout);

  auto status2 = fg2->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
  EXPECT_TRUE(status2 == std::future_status::timeout);

  // Let's S/W trigger from the first -- this could have been a third
  // framegrabber (i.e., client to PCIC)
  fg1->SWTrigger();

  status1 = fg1->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
  EXPECT_TRUE(status1 == std::future_status::ready);
  status2 = fg2->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
  EXPECT_TRUE(status1 == std::future_status::ready);

  // set the camera back into free-run mode
  config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Device::trigger_mode::FREE_RUN));
  dev_->FromJSON(config);
}

TEST_F(FrameGrabberTest, JSON_model)
{
  LOG_INFO("JSON_modelSchema test");
  fg_->Start({ifm3d::buffer_id::JSON_MODEL});
  size_t count = 0;
  fg_->OnNewFrame([&](ifm3d::Frame::Ptr frame) {
    if (frame->HasBuffer(ifm3d::buffer_id::JSON_MODEL))
      {
        fg_->Stop();
      }
    else if (count == 10)
      {
        fg_->Stop();
        EXPECT_TRUE(false);
      }
  });
}

TEST_F(FrameGrabberTest, digonistic_data_grabber)
{
  LOG_INFO("digonistic_data_grabber test");
  EXPECT_NO_THROW(std::make_shared<ifm3d::FrameGrabber>(dev_, 50009));
}
