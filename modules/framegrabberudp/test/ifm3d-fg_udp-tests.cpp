#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <ifm3d/fg.h>
#include <ifm3d/fg_udp.h>
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

TEST(FrameGrabberUdp, CheckEnvironment)
{
  auto cam = ifm3d::Camera::MakeShared();
  if (cam->IsO3X() ||
      !cam->CheckMinimumFirmwareVersion(ifm3d::O3D_UDP_SUPPORT_MAJOR,
                                        ifm3d::O3D_UDP_SUPPORT_MINOR,
                                        ifm3d::O3D_UDP_SUPPORT_PATCH))
    {
      // Test not supported
      return;
    }

  EXPECT_STRNE("", ifm3d::DEFAULT_UDP_TARGET_IP.c_str()) <<
    "ERROR: env var IFM3D_UDP_TARGET_IP must be set for this unit test";
}

TEST(FrameGrabberUdp, FactoryDefaults)
{
  LOG(INFO) << "FactoryDefaults test";
  auto cam = ifm3d::Camera::MakeShared();

  if (cam->IsO3X() ||
      !cam->CheckMinimumFirmwareVersion(ifm3d::O3D_UDP_SUPPORT_MAJOR,
                                        ifm3d::O3D_UDP_SUPPORT_MINOR,
                                        ifm3d::O3D_UDP_SUPPORT_PATCH))
    {
      // Test not supported
      return;
    }


  EXPECT_NO_THROW(cam->FactoryReset());
  std::this_thread::sleep_for(std::chrono::seconds(6));
  EXPECT_NO_THROW(cam->DeviceType());
}

TEST(FrameGrabberUdp, WaitForFrame)
{
  LOG(INFO) << "WaitForFrame test";
  auto cam = ifm3d::Camera::MakeShared();

  if (cam->IsO3X() ||
      !cam->CheckMinimumFirmwareVersion(ifm3d::O3D_UDP_SUPPORT_MAJOR,
                                        ifm3d::O3D_UDP_SUPPORT_MINOR,
                                        ifm3d::O3D_UDP_SUPPORT_PATCH))
    {
      // Test not supported
      return;
    }

  cam->EnableUdp();
  auto fg = std::make_shared<ifm3d::FrameGrabberUdp>();
  auto buff = std::make_shared<MyBuff>();

  int i = 0;
  while (i < 10)
    {
      EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
      i++;
    }

  EXPECT_EQ(i, 10);
}

TEST(FrameGrabberUdp, CustomSchema)
{
  auto cam = ifm3d::Camera::MakeShared();
  if (cam->IsO3X() ||
      !cam->CheckMinimumFirmwareVersion(ifm3d::O3D_UDP_SUPPORT_MAJOR,
                                        ifm3d::O3D_UDP_SUPPORT_MINOR,
                                        ifm3d::O3D_UDP_SUPPORT_PATCH))
    {
      // Test not supported
      return;
    }

  LOG(INFO) << "CustomSchema test";
  std::uint16_t mask = ifm3d::IMG_AMP|ifm3d::IMG_RDIS|ifm3d::IMG_UVEC;

  cam->EnableUdp(mask);
  auto fg = std::make_shared<ifm3d::FrameGrabberUdp>();
  auto buff = std::make_shared<MyBuff>();

  EXPECT_TRUE(fg->WaitForFrame(buff.get(), 1000));
}

