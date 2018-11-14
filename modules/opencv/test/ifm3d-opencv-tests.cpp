#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/opencv.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

template <typename T>
bool cmp_with_nan(T a, T b)
{
  if (std::isnan(a) || std::isnan(b))
    {
      return(std::isnan(a) && std::isnan(b));
    }
  else
    {
      return a == b;
    }
}

TEST(OpenCV, MoveCtor)
{
  LOG(INFO) << "OpenCV.MoveCtor test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::OpenCVBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  cv::Mat amp = im->AmplitudeImage();
  cv::Mat copy_of_amp = amp.clone();

  auto im2 = std::make_shared<ifm3d::OpenCVBuffer>(std::move(*(im.get())));
  cv::Mat amp2 = im2->AmplitudeImage();

  if (cam->IsO3X())
    {
      EXPECT_TRUE(copy_of_amp.type() == CV_32F);
      EXPECT_TRUE(amp2.type() == CV_32F);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<float>(),
                             copy_of_amp.end<float>(),
                             amp2.begin<float>(), cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(copy_of_amp.type() == CV_16UC1);
      EXPECT_TRUE(amp2.type() == CV_16UC1);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<std::uint16_t>(),
                             copy_of_amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }
}

TEST(OpenCV, MoveAssignmentOperator)
{
  LOG(INFO) << "OpenCV.MoveAssignmentOperator test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::OpenCVBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  cv::Mat amp = im->AmplitudeImage();
  cv::Mat copy_of_amp = amp.clone();

  auto im2 = ifm3d::OpenCVBuffer();
  im2 = std::move(*(im.get()));

  cv::Mat amp2 = im2.AmplitudeImage();

  if (cam->IsO3X())
    {
      EXPECT_TRUE(copy_of_amp.type() == CV_32F);
      EXPECT_TRUE(amp2.type() == CV_32F);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<float>(),
                             copy_of_amp.end<float>(),
                             amp2.begin<float>(), cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(copy_of_amp.type() == CV_16UC1);
      EXPECT_TRUE(amp2.type() == CV_16UC1);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<std::uint16_t>(),
                             copy_of_amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }
}

TEST(OpenCV, CopyCtor)
{
  LOG(INFO) << "OpenCV.CopyCtor test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::OpenCVBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  cv::Mat amp = im->AmplitudeImage();

  auto im2 = std::make_shared<ifm3d::OpenCVBuffer>();
  cv::Mat amp2 = im2->AmplitudeImage();
  EXPECT_FALSE((amp.rows * amp.cols) == (amp2.rows * amp2.cols));

  im2 = std::make_shared<ifm3d::OpenCVBuffer>(*(im.get()));
  amp2 = im2->AmplitudeImage();
  EXPECT_TRUE((amp.rows * amp.cols) == (amp2.rows * amp2.cols));

  if (cam->IsO3X())
    {
      EXPECT_TRUE(amp.type() == CV_32F);
      EXPECT_TRUE(amp2.type() == CV_32F);
      EXPECT_TRUE(std::equal(amp.begin<float>(), amp.end<float>(),
                             amp2.begin<float>(), cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(amp.type() == CV_16UC1);
      EXPECT_TRUE(amp2.type() == CV_16UC1);
      EXPECT_TRUE(std::equal(amp.begin<std::uint16_t>(),
                             amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }

  amp2 += 1;

  if (cam->IsO3X())
    {
      EXPECT_FALSE(std::equal(amp.begin<float>(),
                              amp.end<float>(),
                              amp2.begin<float>(), cmp_with_nan<float>));
    }
  else
    {
      EXPECT_FALSE(std::equal(amp.begin<std::uint16_t>(),
                              amp.end<std::uint16_t>(),
                              amp2.begin<std::uint16_t>()));
    }
}

TEST(OpenCV, CopyAssignmentOperator)
{
  LOG(INFO) << "OpenCV.CopyAssignmentOperator test";

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::OpenCVBuffer>();
  auto im2 = std::make_shared<ifm3d::OpenCVBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  *(im2.get()) = *(im.get());

  auto amp = im->AmplitudeImage();
  auto amp2 = im2->AmplitudeImage();

  if (cam->IsO3X())
    {
      EXPECT_TRUE(amp.type() == CV_32F);
      EXPECT_TRUE(amp2.type() == CV_32F);
      EXPECT_TRUE(std::equal(amp.begin<float>(),
                             amp.end<float>(),
                             amp2.begin<float>(), cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(amp.type() == CV_16UC1);
      EXPECT_TRUE(amp2.type() == CV_16UC1);
      EXPECT_TRUE(std::equal(amp.begin<std::uint16_t>(),
                             amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }

  amp2 += 1;

  if (cam->IsO3X())
    {
      EXPECT_FALSE(std::equal(amp.begin<float>(),
                              amp.end<float>(),
                              amp2.begin<float>(), cmp_with_nan<float>));
    }
  else
    {
      EXPECT_FALSE(std::equal(amp.begin<std::uint16_t>(),
                              amp.end<std::uint16_t>(),
                              amp2.begin<std::uint16_t>()));
    }
}

TEST(OpenCV, References)
{
  LOG(INFO) << "OpenCV.References test";

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::OpenCVBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  auto amp1 = im->AmplitudeImage();
  auto amp2 = im->AmplitudeImage();

  if (cam->IsO3X())
    {
      EXPECT_TRUE(amp1.type() == CV_32F);
      EXPECT_TRUE(amp2.type() == CV_32F);
      EXPECT_TRUE(std::equal(amp1.begin<float>(),
                             amp1.end<float>(),
                             amp2.begin<float>(), cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(amp1.type() == CV_16UC1);
      EXPECT_TRUE(amp2.type() == CV_16UC1);
      EXPECT_TRUE(std::equal(amp1.begin<std::uint16_t>(),
                             amp1.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }

  amp2 += 1;

  if (cam->IsO3X())
    {
      EXPECT_TRUE(std::equal(amp1.begin<float>(),
                             amp1.end<float>(),
                             amp2.begin<float>(), cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(std::equal(amp1.begin<std::uint16_t>(),
                             amp1.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }
}

TEST(OpenCV, ComputeCartesian)
{
  //
  // NOTE: The conversions back/forth from/to m/mm is to
  // make this code work for all ifm3d sensors -- i.e.,
  // some return their data as float in meters, others as
  // integers in mm. We normalize to mm and do integer comparisions for
  // correctness.
  //

  auto cam = ifm3d::Camera::MakeShared();
  auto im = std::make_shared<ifm3d::OpenCVBuffer>();

  //
  // 1. Stream in the unit vectors
  //
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_UVEC);
  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));
  cv::Mat uvec = im->UnitVectors();

  //
  // 2. Now we stream in both the radial distance image and the cartesian
  // data. The latter we simply use as ground truth
  //
  fg.reset();
  fg = std::make_shared<ifm3d::FrameGrabber>(
         cam, ifm3d::IMG_RDIS|ifm3d::IMG_CART);
  EXPECT_TRUE(fg->WaitForFrame(im.get(), 1000));
  cv::Mat rdis = im->DistanceImage();
  cv::Mat conf = im->ConfidenceImage();
  cv::Mat xyz = im->XYZImage(); // ground truth

  std::vector<cv::Mat> chans(3);
  cv::split(xyz, chans);
  cv::Mat x_cam, y_cam, z_cam;
  if (chans[0].type() == CV_32FC1)
    {
      // convert to mm
      chans[0] *= 1000.;
      chans[1] *= 1000.;
      chans[2] *= 1000.;

      // cast to int16_t
      chans[0].convertTo(x_cam, CV_16SC1);
      chans[1].convertTo(y_cam, CV_16SC1);
      chans[2].convertTo(z_cam, CV_16SC1);
    }
  else
    {
      // camera data are already mm
      x_cam = chans[0];
      y_cam = chans[1];
      z_cam = chans[2];
    }

  //
  // We also need the translation vector from the extrinsics
  //
  std::vector<float> extrinsics = im->Extrinsics();
  float tx = extrinsics[0];
  float ty = extrinsics[1];
  float tz = extrinsics[2];

  //
  // 3. Compute the cartesian data
  //

  // unit vectors
  cv::Mat ex, ey, ez;
  std::vector<cv::Mat> uvec_chans(3);
  cv::split(uvec, uvec_chans);
  ex = uvec_chans[0];
  ey = uvec_chans[1];
  ez = uvec_chans[2];

  cv::Mat rdis_f;
  rdis.convertTo(rdis_f, CV_32FC1);
  if (rdis.type() == CV_32FC1)
    {
      // assume rdis was in meters, convert to mm
      rdis_f *= 1000.;
    }

  // compute
  cv::Mat x_ = ex.mul(rdis_f) + tx;
  cv::Mat y_ = ey.mul(rdis_f) + ty;
  cv::Mat z_ = ez.mul(rdis_f) + tz;

  // blank out bad pixels ... our zero pixels will
  // be exactly equal to tx, ty, tz and if any of those
  // exceed 1cm (our test tolerance) like on an O3D301,
  // we will get errors in the unit test.
  cv::Mat bad_mask;
  cv::bitwise_and(conf, 0x1, bad_mask);
  x_.setTo(0., bad_mask);
  y_.setTo(0., bad_mask);
  z_.setTo(0., bad_mask);

  //
  // 4. Cast (back) to int16 and transform to ifm3d coord frame
  //
  cv::Mat x_i, y_i, z_i;
  x_.convertTo(x_i, CV_16SC1);
  y_.convertTo(y_i, CV_16SC1);
  z_.convertTo(z_i, CV_16SC1);

  cv::Mat x_computed = z_i;
  cv::Mat y_computed = -x_i;
  cv::Mat z_computed = -y_i;

  //
  // 5. Compare for correctness
  //
  auto cmp = [](std::int16_t a, std::int16_t b) -> bool
    {
      if (std::abs(a - b) <= 10) // 10 mm == cm accuracy
        {
          return true;
        }

      return false;
    };

  EXPECT_TRUE(std::equal(x_cam.begin<std::int16_t>(),
                         x_cam.end<std::int16_t>(),
                         x_computed.begin<std::int16_t>(), cmp));

  EXPECT_TRUE(std::equal(y_cam.begin<std::int16_t>(),
                         y_cam.end<std::int16_t>(),
                         y_computed.begin<std::int16_t>(), cmp));

  EXPECT_TRUE(std::equal(z_cam.begin<std::int16_t>(),
                         z_cam.end<std::int16_t>(),
                         z_computed.begin<std::int16_t>(), cmp));
}

TEST(OpenCV, TimeStamp)
{
  std::string json =
    R"(
        {
          "o3d3xx":
          {
            "Device":
            {
              "ActiveApplication": "1"
            },
            "Apps":
            [
              {
                "TriggerMode": "1",
                "Index": "1",
                "Imager":
                {
                    "ExposureTime": "5000",
                    "ExposureTimeList": "125;5000",
                    "ExposureTimeRatio": "40",
                    "Type":"under5m_moderate"
                }
              }
           ]
          }
        }
      )";

  ifm3d::Camera::Ptr cam = std::make_shared<ifm3d::Camera>();
  cam->FromJSON(nlohmann::json::parse(json));

  ifm3d::OpenCVBuffer::Ptr img = std::make_shared<ifm3d::OpenCVBuffer>();
  ifm3d::FrameGrabber::Ptr fg =
    std::make_shared<ifm3d::FrameGrabber>(
      cam, ifm3d::IMG_AMP|ifm3d::IMG_CART);

  std::array<ifm3d::TimePointT, 2> tps;
  // get two consecutive timestamps
  for (ifm3d::TimePointT& t : tps)
  {
      EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
      t = img->TimeStamp();
  }
  // the first time point need to be smaller than the second one
  EXPECT_LT(tps[0],tps[1]);
  auto tdiff = std::chrono::duration_cast<std::chrono::milliseconds>(
                 tps[1] - tps[0]).count();
  EXPECT_GT(tdiff,20);
}

TEST(OpenCV, IlluTemp)
{
  ifm3d::Camera::Ptr cam = std::make_shared<ifm3d::Camera>();

  ifm3d::OpenCVBuffer::Ptr img = std::make_shared<ifm3d::OpenCVBuffer>();
  ifm3d::FrameGrabber::Ptr fg =
    std::make_shared<ifm3d::FrameGrabber>(
      cam, ifm3d::DEFAULT_SCHEMA_MASK | ifm3d::ILLU_TEMP);

  ASSERT_TRUE(fg->WaitForFrame(img.get(), 1000));

  // currently not supported on O3X
  if (cam->IsO3X())
    {
      return;
    }

  float illu_temp = img->IlluTemp();

  EXPECT_GT(illu_temp, 10);
  EXPECT_LT(illu_temp, 90);
}
