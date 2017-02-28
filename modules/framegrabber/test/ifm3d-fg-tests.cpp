#include <future>
#include <memory>
#include <string>
#include <ifm3d/fg.h>
#include <ifm3d/camera/camera.h>
#include <gtest/gtest.h>

TEST(FrameGrabber, WaitForFrame)
{
  auto cam = std::make_shared<ifm3d::Camera>();
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
  std::uint16_t mask = ifm3d::IMG_AMP|ifm3d::IMG_RDIS|ifm3d::IMG_CART;

  auto cam = std::make_shared<ifm3d::Camera>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, mask);
  auto buff = std::make_shared<ifm3d::ByteBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
}

TEST(FrameGrabber, ByteBufferCopyCtor)
{
  auto cam = std::make_shared<ifm3d::Camera>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff1 = std::make_shared<ifm3d::ByteBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(buff1.get(), 1000));

  auto buff2 = std::make_shared<ifm3d::ByteBuffer>(*(buff1.get()));

  EXPECT_TRUE(buff1->Dirty() != buff2->Dirty());
  EXPECT_TRUE(buff1->Bytes() == buff2->Bytes());
}

TEST(FrameGrabber, ByteBufferCopyAssignmentOperator)
{
  auto cam = std::make_shared<ifm3d::Camera>();
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
  auto cam = std::make_shared<ifm3d::Camera>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto buff = std::make_shared<ifm3d::ByteBuffer>();

  for (int i = 0; i < 5; ++i)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
    }

  fg.reset(new ifm3d::FrameGrabber(cam));
  for (int i = 0; i < 5; ++i)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
    }
}

TEST(FrameGrabber, SoftwareTrigger)
{
  auto cam = std::make_shared<ifm3d::Camera>();

  if (cam->ArticleNumber() == ifm3d::ARTICLE_NUM_O3X)
    {
      return;
    }

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
  auto cam = std::make_shared<ifm3d::Camera>();

  if (cam->ArticleNumber() == ifm3d::ARTICLE_NUM_O3X)
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
