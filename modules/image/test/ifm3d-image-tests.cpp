#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

TEST(Image, MoveCtor)
{
  LOG(INFO) << "Image.MoveCtor test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  cv::Mat amp = im->AmplitudeImage();
  cv::Mat copy_of_amp = amp.clone();

  auto im2 = std::make_shared<ifm3d::ImageBuffer>(std::move(*(im.get())));
  cv::Mat amp2 = im2->AmplitudeImage();

  if (cam->IsO3X())
    {
      EXPECT_TRUE(copy_of_amp.type() == CV_32F);
      EXPECT_TRUE(amp2.type() == CV_32F);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<float>(),
                             copy_of_amp.end<float>(),
                             amp2.begin<float>()));
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

TEST(Image, MoveAssignmentOperator)
{
  LOG(INFO) << "Image.MoveAssignmentOperator test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  cv::Mat amp = im->AmplitudeImage();
  cv::Mat copy_of_amp = amp.clone();

  auto im2 = ifm3d::ImageBuffer();
  im2 = std::move(*(im.get()));

  cv::Mat amp2 = im2.AmplitudeImage();

  if (cam->IsO3X())
    {
      EXPECT_TRUE(copy_of_amp.type() == CV_32F);
      EXPECT_TRUE(amp2.type() == CV_32F);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<float>(),
                             copy_of_amp.end<float>(),
                             amp2.begin<float>()));
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

TEST(Image, CopyCtor)
{
  LOG(INFO) << "Image.CopyCtor test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  cv::Mat amp = im->AmplitudeImage();

  auto im2 = std::make_shared<ifm3d::ImageBuffer>();
  cv::Mat amp2 = im2->AmplitudeImage();
  EXPECT_FALSE((amp.rows * amp.cols) == (amp2.rows * amp2.cols));

  im2 = std::make_shared<ifm3d::ImageBuffer>(*(im.get()));
  amp2 = im2->AmplitudeImage();
  EXPECT_TRUE((amp.rows * amp.cols) == (amp2.rows * amp2.cols));

  if (cam->IsO3X())
    {
      EXPECT_TRUE(amp.type() == CV_32F);
      EXPECT_TRUE(amp2.type() == CV_32F);
      EXPECT_TRUE(std::equal(amp.begin<float>(), amp.end<float>(),
                             amp2.begin<float>()));
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
                              amp2.begin<float>()));
    }
  else
    {
      EXPECT_FALSE(std::equal(amp.begin<std::uint16_t>(),
                              amp.end<std::uint16_t>(),
                              amp2.begin<std::uint16_t>()));
    }
}

TEST(Image, CopyAssignmentOperator)
{
  LOG(INFO) << "Image.CopyAssignmentOperator test";

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();
  auto im2 = std::make_shared<ifm3d::ImageBuffer>();

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
                             amp2.begin<float>()));
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
                              amp2.begin<float>()));
    }
  else
    {
      EXPECT_FALSE(std::equal(amp.begin<std::uint16_t>(),
                              amp.end<std::uint16_t>(),
                              amp2.begin<std::uint16_t>()));
    }
}

TEST(Image, References)
{
  LOG(INFO) << "Image.References test";

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  auto amp1 = im->AmplitudeImage();
  auto amp2 = im->AmplitudeImage();

  if (cam->IsO3X())
    {
      EXPECT_TRUE(amp1.type() == CV_32F);
      EXPECT_TRUE(amp2.type() == CV_32F);
      EXPECT_TRUE(std::equal(amp1.begin<float>(),
                             amp1.end<float>(),
                             amp2.begin<float>()));
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
                             amp2.begin<float>()));
    }
  else
    {
      EXPECT_TRUE(std::equal(amp1.begin<std::uint16_t>(),
                             amp1.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }
}

TEST(Image, CloudMechanics)
{
  //
  // We run this test b/c PCL is still using Boost
  // smart pointers and not std smart pointers. We
  // want to make sure they work the way we think they
  // do ... mechanically.
  //

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  pcl::PointCloud<ifm3d::PointT>::Ptr cloud = im->Cloud();
  EXPECT_TRUE(cloud.use_count() == 2);

  int width = cloud->width;
  int height = cloud->height;

  // new scope
  {
    auto im2 = std::make_shared<ifm3d::ImageBuffer>();
    EXPECT_TRUE(fg->WaitForFrame(im2.get(), 1000));

    cloud = im2->Cloud();
    EXPECT_FALSE(im->Cloud() == cloud);
    EXPECT_TRUE(cloud.use_count() == 2);
  }

  EXPECT_TRUE(cloud.use_count() == 1);
  EXPECT_TRUE(cloud->width == width);
  EXPECT_TRUE(cloud->height == height);
}

TEST(Image, XYZImage)
{
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  pcl::PointCloud<ifm3d::PointT>::Ptr cloud = im->Cloud();
  cv::Mat xyz = im->XYZImage();

  ASSERT_TRUE((xyz.type() == CV_16SC3) || (xyz.type() == CV_32FC3));

  int npts = xyz.rows * xyz.cols;
  ASSERT_EQ(npts, cloud->points.size());

  std::vector<cv::Mat> chans(3);
  cv::split(xyz, chans);
  cv::Mat x = chans[0];
  cv::Mat y = chans[1];
  cv::Mat z = chans[2];

  int i = 0;
  for (int r = 0; r < xyz.rows; ++r)
    {
      for (int c = 0; c < xyz.cols; ++c, ++i)
        {
          ifm3d::PointT& pt = cloud->points[i];
          if (xyz.type() == CV_16SC3)
            {
              EXPECT_FLOAT_EQ(pt.x * 1000., (float) x.at<std::int16_t>(r,c));
              EXPECT_FLOAT_EQ(pt.y * 1000., (float) y.at<std::int16_t>(r,c));
              EXPECT_FLOAT_EQ(pt.z * 1000., (float) z.at<std::int16_t>(r,c));
            }
          else
            {
              EXPECT_FLOAT_EQ(pt.x, x.at<float>(r,c));
              EXPECT_FLOAT_EQ(pt.y, y.at<float>(r,c));
              EXPECT_FLOAT_EQ(pt.z, z.at<float>(r,c));
            }
        }
    }
}

TEST(Image, ComputeCartesian)
{
  //
  // NOTE: The conversions back/forth from/to m/mm is to
  // make this code work for all ifm3d sensors -- i.e.,
  // some return their data as float in meters, others as
  // integers in mm. We normalize to mm and do integer comparisions for
  // correctness.
  //

  auto cam = ifm3d::Camera::MakeShared();
  auto im = std::make_shared<ifm3d::ImageBuffer>();

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

TEST(Image, TimeStamp)
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

  ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
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

TEST(Image, IlluTemp)
{
  ifm3d::Camera::Ptr cam = std::make_shared<ifm3d::Camera>();

  ifm3d::ImageBuffer::Ptr img = std::make_shared<ifm3d::ImageBuffer>();
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
