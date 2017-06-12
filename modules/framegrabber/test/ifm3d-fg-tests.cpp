#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <ifm3d/fg.h>
#include <ifm3d/camera/camera.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

TEST(FrameGrabber, FactoryDefaults)
{
  LOG(INFO) << "FactoryDefaults test";
  auto cam = ifm3d::Camera::MakeShared();

  EXPECT_NO_THROW(cam->FactoryReset());
  std::this_thread::sleep_for(std::chrono::seconds(6));
  EXPECT_NO_THROW(cam->DeviceType());
}

TEST(FrameGrabber, WaitForFrame)
{
  LOG(INFO) << "WaitForFrame test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff = std::make_shared<ifm3d::ByteBuffer>();

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
  std::uint16_t mask = ifm3d::IMG_AMP|ifm3d::IMG_RDIS|ifm3d::IMG_CART;

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, mask);
  auto buff = std::make_shared<ifm3d::ByteBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
}

TEST(FrameGrabber, ByteBufferCopyCtor)
{
  LOG(INFO) << "ByteBufferCopyCtor test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff1 = std::make_shared<ifm3d::ByteBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(buff1.get(), 1000));

  auto buff2 = std::make_shared<ifm3d::ByteBuffer>(*(buff1.get()));

  EXPECT_TRUE(buff1->Dirty() != buff2->Dirty());
  EXPECT_TRUE(buff1->Bytes() == buff2->Bytes());
}

TEST(FrameGrabber, ByteBufferCopyAssignmentOperator)
{
  LOG(INFO) << "ByteBufferCopyAssignmentOperator test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff1 = std::make_shared<ifm3d::ByteBuffer>();
  auto buff2 = std::make_shared<ifm3d::ByteBuffer>();

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
  auto buff = std::make_shared<ifm3d::ByteBuffer>();

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
  auto buff = std::make_shared<ifm3d::ByteBuffer>();

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

  auto buff1 = std::make_shared<ifm3d::ByteBuffer>();
  auto buff2 = std::make_shared<ifm3d::ByteBuffer>();

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
