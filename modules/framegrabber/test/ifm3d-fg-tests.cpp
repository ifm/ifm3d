
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <ifm3d/fg.h>
#include <ifm3d/camera/camera.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <iostream>

// Test class used to mock up a minimal ByteBuffer implemenation
class MyBuff : public ifm3d::ByteBuffer<MyBuff>
{
public:
  MyBuff() : ifm3d::ByteBuffer<MyBuff>()
  {
    LOG(INFO) << "ctor";
  }

  template <typename T>
  void ImCreate(ifm3d::image_chunk im,
                std::uint32_t fmt,
                std::size_t idx,
                std::uint32_t width,
                std::uint32_t height,
                int nchan,
                std::uint32_t npts,
                const std::vector<std::uint8_t>& bytes)
  {
    LOG(INFO) << "ImCreate: " << (int) im;
  }

  template <typename T>
  void CloudCreate(std::uint32_t fmt,
                   std::size_t xidx,
                   std::size_t yidx,
                   std::size_t zidx,
                   std::uint32_t width,
                   std::uint32_t height,
                   std::uint32_t npts,
                   const std::vector<std::uint8_t>& bytes)
  {
    LOG(INFO) << "CloudCreate";
  }

};

TEST(FrameGrabber, FactoryDefaults)
{
  LOG(INFO) << "FactoryDefaults test";
  auto cam = ifm3d::Camera::MakeShared();

  EXPECT_NO_THROW(cam->FactoryReset());
  std::this_thread::sleep_for(std::chrono::seconds(6));
  EXPECT_NO_THROW(cam->DeviceType());
}

TEST(FrameGrabber, ByteBufferBasics)
{
  auto buff = std::make_shared<MyBuff>();
  EXPECT_FALSE(buff->Dirty());

  std::vector<float> zeros(ifm3d::NUM_INTRINSIC_PARAM, 0.);
  std::vector<float> extrinsics = buff->Extrinsics();
  std::vector<std::uint32_t> exposures = buff->ExposureTimes();
  std::vector<float> intrinsic = buff->Intrinsics();
  std::vector<float> inverseIntrinsic = buff->InverseIntrinsics();

  EXPECT_TRUE(std::equal(intrinsic.begin(), intrinsic.end(), zeros.begin()));
  EXPECT_TRUE(std::equal(inverseIntrinsic.begin(), inverseIntrinsic.end(), zeros.begin()));
  EXPECT_TRUE(std::equal(extrinsics.begin(), extrinsics.end(), zeros.begin()));
  EXPECT_TRUE(std::equal(exposures.begin(), exposures.end(), zeros.begin()));
}

TEST(FrameGrabber, WaitForFrame)
{
  LOG(INFO) << "WaitForFrame test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff = std::make_shared<MyBuff>();

  int i = 0;
  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
      i++;
    }

  EXPECT_EQ(i, 10);
}

TEST(FrameGrabber, CustomSchema)
{
  LOG(INFO) << "CustomSchema test";
  std::uint16_t mask = ifm3d::IMG_AMP|ifm3d::IMG_RDIS|ifm3d::IMG_UVEC;

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, mask);
  auto buff = std::make_shared<MyBuff>();

  EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
}

TEST(FrameGrabber, IntrinsicParamSchema)
{
  LOG(INFO) << "IntrinsicParamSchema test";
  std::uint16_t mask = ifm3d::IMG_AMP|ifm3d::IMG_RDIS|ifm3d::INTR_CAL;

  auto cam = ifm3d::Camera::MakeShared();
  if(cam->IsO3X()) // intrinsic parameter  not supported
   {
     EXPECT_THROW(std::make_shared<ifm3d::FrameGrabber>(cam, mask),
                  ifm3d::error_t);
   }
  if(cam->IsO3D() &&
     cam->CheckMinimumFirmwareVersion(
       ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MAJOR,
       ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_MINOR,
       ifm3d::O3D_INTRINSIC_PARAM_SUPPORT_PATCH))
   {
     auto fg = std::make_shared<ifm3d::FrameGrabber>(cam,mask);
     auto buff = std::make_shared<MyBuff>();
     EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
   }
  else if(cam->IsO3D())
   {
     EXPECT_THROW(std::make_shared<ifm3d::FrameGrabber>(cam, mask),
                  ifm3d::error_t);
   }
}

TEST(FrameGrabber, InverseIntrinsicParamSchema)
{
  LOG(INFO) << "InverseIntrinsicParamSchema test";

  std::uint16_t mask = (ifm3d::IMG_AMP|ifm3d::IMG_RDIS|
                        ifm3d::INTR_CAL|ifm3d::INV_INTR_CAL);
  auto cam = ifm3d::Camera::MakeShared();
  if(cam->IsO3X()) // inverse intrinsic parameter  not supported
   {
     EXPECT_THROW(std::make_shared<ifm3d::FrameGrabber>(cam, mask),
                  ifm3d::error_t);
   }
  if(cam->IsO3D() &&
     cam->CheckMinimumFirmwareVersion(
       ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR,
       ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR,
       ifm3d::O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH))
   {
     auto fg = std::make_shared<ifm3d::FrameGrabber>(cam,mask);
     auto buff = std::make_shared<MyBuff>();

     EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));

     std::vector<float> intrinsics=buff->Intrinsics();
     EXPECT_TRUE(std::accumulate(intrinsics.begin(),
                                 intrinsics.end(), 0.) > 0.);

     std::vector<float> inverseIntrinsics=buff->InverseIntrinsics();
     EXPECT_TRUE(std::accumulate(inverseIntrinsics.begin(),
                                 inverseIntrinsics.end(), 0.) > 0.);
   }
  else if(cam->IsO3D())
   {
     EXPECT_THROW(std::make_shared<ifm3d::FrameGrabber>(cam, mask),
                  ifm3d::error_t);
   }
}

TEST(FrameGrabber, ByteBufferMoveCtor)
{
  LOG(INFO) << "ByteBufferMoveCtor test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff1 = std::make_shared<MyBuff>();

  EXPECT_TRUE(fg->WaitForFrame(buff1.get(), 1000));

  auto copy_of_buff1_bytes = buff1->Bytes();
  auto buff2 = std::make_shared<MyBuff>(std::move(*(buff1.get())));

  EXPECT_TRUE(copy_of_buff1_bytes == buff2->Bytes());
}

TEST(FrameGrabber, ByteBufferMoveAssignmentOperator)
{
  LOG(INFO) << "ByteBufferMoveAssignmentOperator test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff1 = std::make_shared<MyBuff>();
  auto buff2 = MyBuff();

  EXPECT_TRUE(fg->WaitForFrame(buff1.get(), 1000));

  auto copy_of_buff1_bytes = buff1->Bytes();
  buff2 = std::move(*(buff1.get()));

  EXPECT_TRUE(copy_of_buff1_bytes == buff2.Bytes());
}

TEST(FrameGrabber, ByteBufferCopyCtor)
{
  LOG(INFO) << "ByteBufferCopyCtor test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff1 = std::make_shared<MyBuff>();

  EXPECT_TRUE(fg->WaitForFrame(buff1.get(), 1000));

  auto buff2 = std::make_shared<MyBuff>(*(buff1.get()));

  EXPECT_TRUE(buff1->Dirty() != buff2->Dirty());
  EXPECT_TRUE(buff1->Bytes() == buff2->Bytes());
}

TEST(FrameGrabber, ByteBufferCopyAssignmentOperator)
{
  LOG(INFO) << "ByteBufferCopyAssignmentOperator test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff1 = std::make_shared<MyBuff>();
  auto buff2 = std::make_shared<MyBuff>();

  EXPECT_TRUE(fg->WaitForFrame(buff1.get(), 1000));

  *(buff2.get()) = *(buff1.get());
  EXPECT_TRUE(buff1->Dirty() != buff2->Dirty());
  EXPECT_TRUE(buff1->Bytes() == buff2->Bytes());
}

TEST(FrameGrabber, FrameGrabberRecycling)
{
  LOG(INFO) << "FrameGrabberRecycling test";

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff = std::make_shared<MyBuff>();

  for (int i = 0; i < 5; ++i)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
    }

  if (! cam->IsO3X())
    {
      fg.reset(new ifm3d::FrameGrabber(cam));
      for (int i = 0; i < 5; ++i)
        {
          EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
        }
    }
  else
    {
      //
      // O3X PCIC can only handle a single connection
      // so, resetting the framegrabber needs to happen
      // in discrete steps:
      //
      // 1. Ensure the first fg's dtor is run which joins
      //    on the framegrabbing thread holding the open socket.
      // 2. Instantiate the new fg - which creates a new thread
      //    and opens a new socket.
      //
      fg.reset();
      fg = std::make_shared<ifm3d::FrameGrabber>(cam);
      for (int i = 0; i < 5; ++i)
        {
          EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
        }
    }
}

TEST(FrameGrabber, SoftwareTrigger)
{
  LOG(INFO) << "SoftwareTrigger test";
  auto cam = ifm3d::Camera::MakeShared();

  // mark the current active application as sw triggered
  int idx = cam->ActiveApplication();
  json config = cam->ToJSON();
  config["ifm3d"]["Apps"][idx-1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Camera::trigger_mode::SW));
  cam->FromJSON(config);

  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff = std::make_shared<MyBuff>();

  // waiting for an image should now timeout
  EXPECT_FALSE(fg->WaitForFrame(buff.get(), 1000));

  // now, get image data by explicitly s/w triggering the device
  for (int i = 0; i < 10; ++i)
    {
      fg->SWTrigger();
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));

    }

  // set the camera back into free-run mode
  config["ifm3d"]["Apps"][idx-1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Camera::trigger_mode::FREE_RUN));
  cam->FromJSON(config);
}

TEST(FrameGrabber, SWTriggerMultipleClients)
{
  LOG(INFO) << "SWTriggerMultipleClients test";
  auto cam = ifm3d::Camera::MakeShared();

  //
  // O3X cannot handle multiple client connections to PCIC
  // so this test does not apply
  //
  if (cam->IsO3X())
    {
      return;
    }

  // mark the current active application as sw triggered
  int idx = cam->ActiveApplication();
  json config = cam->ToJSON();
  config["ifm3d"]["Apps"][idx-1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Camera::trigger_mode::SW));
  cam->FromJSON(config);

  // create two framegrabbers and two buffers
  auto fg1 = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto fg2 = std::make_shared<ifm3d::FrameGrabber>(cam);

  auto buff1 = std::make_shared<MyBuff>();
  auto buff2 = std::make_shared<MyBuff>();

  // launch two threads where each of the framegrabbers will
  // wait for a new frame
  auto fut1 = std::async(std::launch::async,
                         [fg1,buff1]()->bool
                         {return fg1->WaitForFrame(buff1.get(),5000);});
  auto fut2 = std::async(std::launch::async,
                         [fg2,buff2]()->bool
                         {return fg2->WaitForFrame(buff2.get(),5000);});

  // Let's S/W trigger from the first -- this could have been a third
  // framegrabber (i.e., client to PCIC)
  fg1->SWTrigger();

  // Did they both get a frame?
  ASSERT_TRUE(fut1.get());
  ASSERT_TRUE(fut2.get());

  // Check that the data are the same
  EXPECT_TRUE(buff1->Bytes() == buff2->Bytes());

  // set the camera back into free-run mode
  config["ifm3d"]["Apps"][idx-1]["TriggerMode"] =
    std::to_string(static_cast<int>(ifm3d::Camera::trigger_mode::FREE_RUN));
  cam->FromJSON(config);
}

TEST(FrameGrabber, JSON_model)
{
  LOG(INFO) << "JSON_modelSchema test";

  std::uint16_t mask = (ifm3d::IMG_AMP | ifm3d::JSON_MODEL);
  auto cam = ifm3d::Camera::MakeShared();
  if (cam->IsO3X()) // JSON model not supported
    {
      EXPECT_THROW(std::make_shared<ifm3d::FrameGrabber>(cam, mask),
        ifm3d::error_t);
    }
  else if (cam->IsO3D())
    {
      auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, mask);
      auto buff = std::make_shared<MyBuff>();

      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
      std::string model = "";
      EXPECT_NO_THROW(model = buff->JSONModel());
      // checking is its a valid model 
      EXPECT_TRUE(json::parse(model) != NULL);
    }
}
