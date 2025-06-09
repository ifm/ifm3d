#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <ifm3d/common/err.h>
#include <ifm3d/common/json_impl.hpp>
#include <ifm3d/common/logging/log.h>
#include <ifm3d/device/device.h>
#include <ifm3d/device/legacy_device.h>
#include <ifm3d/device/o3r.h>
#include <ifm3d/fg/buffer_id.h>
#include <ifm3d/fg/frame.h>
#include <ifm3d/fg/frame_grabber.h>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

constexpr uint16_t O3R_PORT = 50012;

class FrameGrabberTest : public ::testing::Test
{
protected:
  void
  SetUp() override
  {
    this->_dev = ifm3d::Device::MakeShared();
    if (_dev->WhoAmI() == ifm3d::Device::DeviceFamily::O3R)
      {
        auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->_dev);

        auto config = o3r->Get();
        config["ports"]["port2"]["state"] = "RUN";
        o3r->Set(config);
        _fg = std::make_shared<ifm3d::FrameGrabber>(_dev, O3R_PORT);
      }
    else
      {
        _fg = std::make_shared<ifm3d::FrameGrabber>(_dev);
      }
  }

  void
  TearDown() override
  {}

  // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
  ifm3d::Device::Ptr _dev;
  // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
  ifm3d::FrameGrabber::Ptr _fg;
};
#if 0 // NOLINT(readability-avoid-unconditional-preprocessor-if)
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
    std::dynamic_pointer_cast<ifm3d::LegacyDevice>(_dev)->FactoryReset());
  std::this_thread::sleep_for(std::chrono::seconds(6));
  EXPECT_NO_THROW(this->_dev->DeviceType());
}

TEST_F(FrameGrabberTest, WaitForFrame)
{
  LOG_INFO("WaitForFrame test");
  this->_fg->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
                    ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
                    ifm3d::buffer_id::XYZ});
  int i = 0;
  while (i < 10)
    {
      EXPECT_NO_THROW(this->_fg->WaitForFrame().get());
      i++;
    }

  EXPECT_EQ(i, 10);
}

TEST_F(FrameGrabberTest, CustomSchema)
{
  LOG_INFO("CustomSchema test");

  _fg->Start({ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ});

  auto frame = _fg->WaitForFrame().get();

  auto amplitude = frame->GetBuffer(ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE);
  auto distance = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
  auto xyz = frame->GetBuffer(ifm3d::buffer_id::XYZ);

  EXPECT_TRUE(amplitude.Width() != 0);
  EXPECT_TRUE(distance.Width() != 0);
  EXPECT_TRUE(xyz.NumChannels() == 3);
}

TEST_F(FrameGrabberTest, BlankSchema3D)
{
  LOG_INFO("BlankSchema3D test");

  _fg->Start({});

  auto frame = _fg->WaitForFrame().get();

  auto distance = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
  auto xyz = frame->GetBuffer(ifm3d::buffer_id::XYZ);

  EXPECT_TRUE(distance.Width() != 0);
  EXPECT_TRUE(xyz.NumChannels() == 3);
}

TEST_F(FrameGrabberTest, BlankSchema2D)
{
  LOG_INFO("BlankSchema2D test");

  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->_dev);

  auto config = o3r->Get();
  config["ports"]["port0"]["state"] = "RUN";
  o3r->Set(config);
  _fg = std::make_shared<ifm3d::FrameGrabber>(_dev, 50010);

  _fg->Start({});

  auto frame = _fg->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::JPEG_IMAGE));
  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::RGB_INFO));
}

TEST_F(FrameGrabberTest, schema_o3r_rgb_image_info)
{
  LOG_INFO("schema_o3r_rgb_image_info test");

  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->_dev);

  auto config = o3r->Get();
  config["ports"]["port0"]["state"] = "RUN";
  o3r->Set(config);
  _fg = std::make_shared<ifm3d::FrameGrabber>(_dev, 50010);

  _fg->Start({ifm3d::buffer_id::RGB_INFO});

  auto frame = _fg->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::RGB_INFO));
}

TEST_F(FrameGrabberTest, schema_o3r_dist_image_info)
{
  LOG_INFO("schema_o3r_dist_image_info test");

  _fg->Start({ifm3d::buffer_id::TOF_INFO});

  auto frame = _fg->WaitForFrame().get();

  EXPECT_NO_THROW(auto o3r_dist_image_info =
                    frame->GetBuffer(ifm3d::buffer_id::TOF_INFO));
}

TEST_F(FrameGrabberTest, BufferIDException)
{
  LOG_INFO("BufferIDException test");

  _fg->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ});

  auto frame = _fg->WaitForFrame().get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE));
  EXPECT_THROW(frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE),
               ifm3d::Error);
}

TEST_F(FrameGrabberTest, DistanceNoiseImageSchema)
{
  LOG_INFO(" distance noise image schema test");

  _fg->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
              ifm3d::buffer_id::XYZ});

  auto frame = _fg->WaitForFrame().get();

  // as not part of schema
  EXPECT_ANY_THROW(frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE));
}

TEST_F(FrameGrabberTest, DistanceNoiseImage_type)
{
  LOG_INFO(" distance noise image test");

  _fg
    ->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
             ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
             ifm3d::buffer_id::RADIAL_DISTANCE_NOISE})
    .wait_for(std::chrono::seconds(2));
  size_t count = 0;
  _fg->OnNewFrame([&](const ifm3d::Frame::Ptr& frame) {
    if (frame->HasBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE))
      {
        count++;
        auto distance_noise_image =
          frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE);
        if (_dev->AmI(ifm3d::Device::DeviceFamily::O3R))
          {
            EXPECT_EQ(distance_noise_image.DataFormat(),
                      ifm3d::PixelFormat::FORMAT_32F);
          }
        if (_dev->AmI(ifm3d::Device::DeviceFamily::O3X))
          {
            EXPECT_EQ(distance_noise_image.DataFormat(),
                      ifm3d::PixelFormat::FORMAT_16U);
          }
        _fg->Stop();
      }
    else if (count == 10)
      {
        _fg->Stop();
        EXPECT_TRUE(false);
      }
  });
}

TEST_F(FrameGrabberTest, onError)
{
  LOG_INFO(" onError ");

  auto result = 0;
  auto frame_count = 0;

  _fg->OnError([&](const ifm3d::Error& /*err*/) { result++; });

  _fg->OnNewFrame([&](const ifm3d::Frame::Ptr& /*frame*/) { frame_count++; });

  _fg->Start({ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
              ifm3d::buffer_id::CONFIDENCE_IMAGE});

  std::this_thread::sleep_for(std::chrono::seconds(10));

  // this reboot will cause network interruption and onError must get error
  _dev->Reboot();

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

  _fg
    ->Start({ifm3d::buffer_id::AMPLITUDE_IMAGE,
             ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
             ifm3d::buffer_id::XYZ,
             ifm3d::buffer_id::CONFIDENCE_IMAGE})
    .wait_for(std::chrono::seconds(1));
  size_t count = 0;
  _fg->OnNewFrame([&](const ifm3d::Frame::Ptr& frame) {
    count++;
    if (frame->HasBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE))
      {

        EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE));

        auto confidence_image =
          frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE);

        if (_dev->AmI(ifm3d::Device::DeviceFamily::O3R))
          {
            EXPECT_EQ(confidence_image.DataFormat(),
                      ifm3d::PixelFormat::FORMAT_16U);
          }
        else
          {
            EXPECT_EQ(confidence_image.DataFormat(),
                      ifm3d::PixelFormat::FORMAT_8U);
          }
        _fg->Stop();
      }
    else if (count == 10)
      {
        _fg->Stop();
        EXPECT_TRUE(false);
      }
  });
}

TEST_F(FrameGrabberTest, confidence_image_2D)
{
  LOG_INFO(" confidence image test on 2D  ");

  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->_dev);

  auto config = o3r->Get();
  config["ports"]["port0"]["state"] = "RUN";
  o3r->Set(config);
  _fg = std::make_shared<ifm3d::FrameGrabber>(_dev, 50010);

  _fg->Start(
    {ifm3d::buffer_id::JPEG_IMAGE, ifm3d::buffer_id::CONFIDENCE_IMAGE});

  auto frame = _fg->WaitForFrame().get();

  EXPECT_THROW(frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE),
               ifm3d::Error);
}

TEST_F(FrameGrabberTest, only_algo_debug)
{
  LOG_INFO(" obtain only algo debug data");
  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->_dev);

  // enable algo debug flag through xmlrpc set interface
  o3r->Reset("/ports/port2/data/algoDebugFlag");
  o3r->Set({{"ports", {{"port2", {{"data", {{"algoDebugFlag", true}}}}}}}});

  std::this_thread::sleep_for(std::chrono::seconds(1));

  _fg->Start({ifm3d::buffer_id::ALGO_DEBUG});

  auto future = _fg->WaitForFrame();
  auto status = future.wait_for(std::chrono::seconds(1));
  EXPECT_TRUE(status == std::future_status::ready);

  const auto& frame = future.get();
  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::ALGO_DEBUG));
  EXPECT_THROW(frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE),
               ifm3d::Error);
}

TEST_F(FrameGrabberTest, algo_with_other_data)
{
  LOG_INFO(" obtain  algo debug with other data");
  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->_dev);

  // enable algo debug flag through xmlrpc set interface
  o3r->Reset("/ports/port2/data/algoDebugFlag");
  o3r->Set({{"ports", {{"port2", {{"data", {{"algoDebugFlag", true}}}}}}}});

  std::this_thread::sleep_for(std::chrono::seconds(1));

  _fg->Start(
    {ifm3d::buffer_id::ALGO_DEBUG, ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE});

  for (int i = 0; i < 20; i++)
    {
      auto frame = _fg->WaitForFrame().get();

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
  _fg->OnError([&has_error](const ifm3d::Error& e) {
    LOG_ERROR(e.what());
    has_error = true;
  });

  for (int i = 0; i < 3; ++i)
    {
      EXPECT_FALSE(_fg->IsRunning());

      EXPECT_TRUE(_fg->Start({}).wait_for(std::chrono::seconds(5)) ==
                  std::future_status::ready);

      EXPECT_TRUE(_fg->WaitForFrame().wait_for(std::chrono::seconds(1)) ==
                  std::future_status::ready);

      EXPECT_TRUE(_fg->IsRunning());

      EXPECT_TRUE(_fg->Stop().wait_for(std::chrono::seconds(5)) ==
                  std::future_status::ready);

      EXPECT_FALSE(_fg->IsRunning());
    }

  EXPECT_FALSE(has_error);
}

TEST_F(FrameGrabberTest, FrameGrabberRecycling)
{
  LOG_INFO("FrameGrabberRecycling test");
  _fg->Start({});

  for (int i = 0; i < 5; ++i)
    {
      auto status =
        _fg->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
      EXPECT_TRUE(status == std::future_status::ready);
    }
  _fg.reset();
  if (_dev->WhoAmI() == ifm3d::Device::DeviceFamily::O3R)
    {
      auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->_dev);

      auto config = o3r->Get();
      config["ports"]["port2"]["state"] = "RUN";
      o3r->Set(config);
      _fg = std::make_shared<ifm3d::FrameGrabber>(_dev, O3R_PORT);
    }
  else
    {
      _fg = std::make_shared<ifm3d::FrameGrabber>(_dev);
    }
  _fg->Start({});
  for (int i = 0; i < 5; ++i)
    {
      auto status =
        _fg->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
      EXPECT_TRUE(status == std::future_status::ready);
    }
}

TEST_F(FrameGrabberTest, SoftwareTrigger)
{
  LOG_INFO("SoftwareTrigger test");

  auto legacy_device = std::dynamic_pointer_cast<ifm3d::LegacyDevice>(_dev);
  // mark the current active application as sw triggered
  int const idx = legacy_device->ActiveApplication();
  ifm3d::json config = legacy_device->ToJSON();
  config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Device::TriggerMode::SW));
  legacy_device->FromJSON(config);

  _fg->Start({});

  // waiting for an image should now timeout
  auto status = _fg->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
  EXPECT_TRUE(status == std::future_status::timeout);

  // now, get image data by explicitly s/w triggering the device
  for (int i = 0; i < 10; ++i)
    {
      _fg->SWTrigger();
      auto status =
        _fg->WaitForFrame().wait_for(std::chrono::milliseconds(1000));
      EXPECT_TRUE(status == std::future_status::ready);
    }

  // set the camera back into free-run mode
  config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Device::TriggerMode::FREE_RUN));
  _dev->FromJSON(config);
}

TEST_F(FrameGrabberTest, SWTriggerMultipleClients)
{
  LOG_INFO("SWTriggerMultipleClients test");

  auto legacy_device = std::dynamic_pointer_cast<ifm3d::LegacyDevice>(_dev);
  // mark the current active application as sw triggered
  int const idx = legacy_device->ActiveApplication();
  ifm3d::json config = legacy_device->ToJSON();
  config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Device::TriggerMode::SW));
  legacy_device->FromJSON(config);

  // create two framegrabbers and two buffers
  auto fg1 = std::make_shared<ifm3d::FrameGrabber>(legacy_device, 50010);
  auto fg2 = _fg;

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
  EXPECT_TRUE(status2 == std::future_status::ready);

  // set the camera back into free-run mode
  config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Device::TriggerMode::FREE_RUN));
  _dev->FromJSON(config);
}

TEST_F(FrameGrabberTest, JSON_model)
{
  LOG_INFO("JSON_modelSchema test");
  _fg->Start({ifm3d::buffer_id::JSON_MODEL});
  size_t count = 0;
  _fg->OnNewFrame([&](const ifm3d::Frame::Ptr& frame) {
    if (frame->HasBuffer(ifm3d::buffer_id::JSON_MODEL))
      {
        _fg->Stop();
      }
    else if (count == 10)
      {
        _fg->Stop();
        EXPECT_TRUE(false);
      }
  });
}

TEST_F(FrameGrabberTest, digonistic_data_grabber)
{
  LOG_INFO("digonistic_data_grabber test");
  EXPECT_NO_THROW(std::make_shared<ifm3d::FrameGrabber>(_dev, 50009));
}

TEST_F(FrameGrabberTest, metadata)
{
  LOG_INFO("PDS_CHUNKS test");
  using namespace ifm3d::literals;

  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->_dev);

  // setup device for PDS application
  ifm3d::json const json_command_extrinsic =
    R"({ "ports":{"port2":{"processing":{"extrinsicHeadToUser":{"rotX":0.0047429079134991,"rotY" :2.110169438134602,"rotZ" :-1.5899949128649966 ,"transX" : 0.269,"transY" : -0.163,"transZ" : 0.514}}}} })"_json;

  o3r->Set(json_command_extrinsic);

  ifm3d::json const json_command_test_method =
    R"({"applications":{"instances":{"app0":{"class":"pds", "ports" : ["port2"] , "state" : "IDLE", "configuration" : {"parameter":{"testMode":1}}}}}})"_json;
  o3r->Set(json_command_test_method);

  const auto fg_pcic_port =
    o3r->Get()["/applications/instances/app0/data/pcicTCPPort"_json_pointer];

  auto fg = std::make_shared<ifm3d::FrameGrabber>(o3r, fg_pcic_port);

  // Set Schema and start the grabber
  fg->Start({ifm3d::buffer_id::O3R_RESULT_JSON,
             ifm3d::buffer_id::O3R_RESULT_ARRAY2D})
    .wait();

  auto current_config = o3r->Get();
  std::string const current_command = current_config
    ["/applications/instances/app0/configuration/customization/command"_json_pointer];

  auto future = fg->WaitForFrame();

  if (current_command == "nop")
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      ifm3d::json const json_command_get_pallet =
        R"({ "applications":{"instances":{"app0":{"configuration":{"customization":{"command":"getPallet"}}}}} })"_json;

      o3r->Set(json_command_get_pallet);
    }

  const auto& frame = future.get();

  EXPECT_NO_THROW(frame->GetBuffer(ifm3d::buffer_id::O3R_RESULT_JSON));

  auto buffer = frame->GetBuffer(
    static_cast<ifm3d::buffer_id>(ifm3d::buffer_id::O3R_RESULT_JSON));

  EXPECT_TRUE(!buffer.Metadata().empty());
  EXPECT_TRUE(frame->GetBufferCount(static_cast<ifm3d::buffer_id>(
                ifm3d::buffer_id::O3R_RESULT_JSON)) > 0);
}

TEST_F(FrameGrabberTest, buffer_mapping)
{
  auto o3r = std::dynamic_pointer_cast<ifm3d::O3R>(this->dev);
  auto fg = std::make_shared<ifm3d::FrameGrabber>(o3r, 51010);
  fg->Start({
    ifm3d::buffer_id::O3R_ODS_RENDERED_ZONES,
  });
  auto frame = fg->WaitForFrame().get();
  auto& buffer = frame->GetBuffer(ifm3d::buffer_id::O3R_ODS_RENDERED_ZONES);
  EXPECT_TRUE(buffer.metadata()["result"]["type"] == "ods_rendered_zones");
}