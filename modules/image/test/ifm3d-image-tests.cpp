#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <gtest/gtest.h>

TEST(Image, CloudMechanics)
{
  //
  // We run this test b/c PCL is still using Boost
  // smart pointers and not std smart pointers. We
  // want to make sure they work the way we think they
  // do ... mechanically.
  //

  auto cam = std::make_shared<ifm3d::Camera>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(im.get(), 1000));

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
  auto cam = std::make_shared<ifm3d::Camera>();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  EXPECT_TRUE(fg->WaitForFrame(im.get(), 1000));

  pcl::PointCloud<ifm3d::PointT>::Ptr cloud = im->Cloud();
  cv::Mat xyz = im->XYZImage();

  // XXX: May fail on O3X ???
  ASSERT_EQ(xyz.type(), CV_16SC3);

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
          EXPECT_FLOAT_EQ(pt.x * 1000., (float) x.at<std::int16_t>(r,c));
          EXPECT_FLOAT_EQ(pt.y * 1000., (float) y.at<std::int16_t>(r,c));
          EXPECT_FLOAT_EQ(pt.z * 1000., (float) z.at<std::int16_t>(r,c));
        }
    }
}

TEST(Image, ComputeCartesian)
{
  auto cam = std::make_shared<ifm3d::Camera>();
  auto im = std::make_shared<ifm3d::ImageBuffer>();

  //
  // 1. Stream in the unit vectors
  //
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_UVEC);
  EXPECT_TRUE(fg->WaitForFrame(im.get(), 1000));
  cv::Mat uvec = im->UnitVectors();

  //
  // 2. Now we stream in both the radial distance image and the cartesian
  // data. The latter we simply use as ground truth
  //
  fg.reset(new ifm3d::FrameGrabber(cam, ifm3d::IMG_RDIS|ifm3d::IMG_CART));
  EXPECT_TRUE(fg->WaitForFrame(im.get(), 1000));
  cv::Mat rdis = im->DistanceImage();
  cv::Mat xyz = im->XYZImage(); // ground truth

  std::vector<cv::Mat> chans(3);
  cv::split(xyz, chans);
  cv::Mat x_cam = chans[0];
  cv::Mat y_cam = chans[1];
  cv::Mat z_cam = chans[2];

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

  // compute
  cv::Mat x_ = ex.mul(rdis_f) + tx;
  cv::Mat y_ = ey.mul(rdis_f) + ty;
  cv::Mat z_ = ez.mul(rdis_f) + tz;

  //
  // 4. Cast back to int16 and transform to ifm3d coord frame
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
  // Recall, the XYZImage from the ImageBuffer is in mm, so, we will consider
  // these values equal if they are within 1 cm of eachother (rounding
  // differences in OpenCV (in the cast from float to int16_t above) vs. the
  // camera is a likely source of such discrepancies).
  //
  auto cmp = [](std::int16_t a, std::int16_t b) -> bool
    {
      if (std::abs(a - b) <= 10) // cm accuracy
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
