#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/stlimage.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

namespace
{
  template <typename T>
  bool
  cmp_with_nan(T a, T b)
  {
    if (std::isnan(a) || std::isnan(b))
      {
        return (std::isnan(a) && std::isnan(b));
      }
    else
      {
        return a == b;
      }
  }

  template <typename T>
  ifm3d::Image
  add(ifm3d::Image& img, T val)
  {
    for (auto& pix_val : ifm3d::IteratorAdapter<T>(img))
      {
        pix_val += val;
      }
    return img;
  }

  template <typename T>
  void
  mul(ifm3d::Image& img, T val)
  {
    for (auto& pix_val : ifm3d::IteratorAdapter<T>(img))
      {
        pix_val *= val;
      }
  }

  template <typename T1, typename T2>
  ifm3d::Image&
  mul(ifm3d::Image &img1, ifm3d::Image &img2)
  {
    auto it = img2.begin<T2>();
    for (auto& pix_val : ifm3d::IteratorAdapter<T1>(img1))
      {
        pix_val *= *it;
        it++;
      }
    return img1;
  }

  template <typename T>
  void
  split(ifm3d::Image& split_this, std::vector<ifm3d::Image>& to_this)
  {
    // this is not a full split function, this function assumes that
    // split_this will have three channel and to_this vector will have
    // 3 ifm3d::Image images

    using p3d = ifm3d::Point3D<T>;

    for (auto& image : to_this)
      {
        image.create(split_this.width(),
                     split_this.height(),
                     1,
                     split_this.dataFormat());
      }

    auto it1 = to_this[0].begin<T>();
    auto it2 = to_this[1].begin<T>();
    auto it3 = to_this[2].begin<T>();

    for (auto value : ifm3d::IteratorAdapter<p3d>(split_this))
      {
        *it1 = value.val[0];
        *it2 = value.val[1];
        *it3 = value.val[2];

        it1++;
        it2++;
        it3++;
      }
  }
  template <typename T1, typename T2>
  void
  convert_to(ifm3d::Image& convert_this,
             ifm3d::Image& to_this,
             ifm3d::pixel_format this_type)
  {
    to_this.create(convert_this.width(), convert_this.height(), 1, this_type);
    auto it = to_this.begin<T2>();
    for (auto& value : ifm3d::IteratorAdapter<T1>(convert_this))
      {
        *it = (T2)value;
        it++;
      }
  }
} // end namespace
TEST(StlImageBuffer, MoveCtor)
{
  LOG(INFO) << "StlImage.MoveCtor test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::StlImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  ifm3d::Image amp = im->AmplitudeImage();
  ifm3d::Image copy_of_amp = amp.clone();

  auto im2 = std::make_shared<ifm3d::StlImageBuffer>(std::move(*(im.get())));
  ifm3d::Image amp2 = im2->AmplitudeImage();

  if (cam->AmI(ifm3d::Camera::device_family::O3X) ||
      cam->AmI(ifm3d::Camera::device_family::O3R))
    {
      EXPECT_TRUE(copy_of_amp.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(copy_of_amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<float>(),
                             copy_of_amp.end<float>(),
                             amp2.begin<float>(),
                             cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(copy_of_amp.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(copy_of_amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<std::uint16_t>(),
                             copy_of_amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }
}

TEST(StlImageBuffer, MoveAssignmentOperator)
{
  LOG(INFO) << "Image.MoveAssignmentOperator test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::StlImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  ifm3d::Image amp = im->AmplitudeImage();
  ifm3d::Image copy_of_amp = amp.clone();

  auto im2 = ifm3d::StlImageBuffer();
  im2 = std::move(*(im.get()));

  ifm3d::Image amp2 = im2.AmplitudeImage();

  if (cam->AmI(ifm3d::Camera::device_family::O3X) ||
      cam->AmI(ifm3d::Camera::device_family::O3R))
    {
      EXPECT_TRUE(copy_of_amp.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(copy_of_amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<float>(),
                             copy_of_amp.end<float>(),
                             amp2.begin<float>(),
                             cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(copy_of_amp.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(copy_of_amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(copy_of_amp.begin<std::uint16_t>(),
                             copy_of_amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }
}

TEST(StlImageBuffer, CopyCtor)
{
  LOG(INFO) << "Image.CopyCtor test";
  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::StlImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  ifm3d::Image amp = im->AmplitudeImage();

  auto im2 = std::make_shared<ifm3d::StlImageBuffer>();
  ifm3d::Image amp2 = im2->AmplitudeImage();

  EXPECT_FALSE((amp.height() * amp.width()) == (amp2.height() * amp2.width()));

  im2 = std::make_shared<ifm3d::StlImageBuffer>(*(im.get()));
  amp2 = im2->AmplitudeImage();
  EXPECT_TRUE((amp.height() * amp.width()) == (amp2.height() * amp2.width()));

  if (cam->AmI(ifm3d::Camera::device_family::O3X) ||
      cam->AmI(ifm3d::Camera::device_family::O3R))
    {
      EXPECT_TRUE(amp.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(amp.begin<float>(),
                             amp.end<float>(),
                             amp2.begin<float>(),
                             cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(amp.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(amp.begin<std::uint16_t>(),
                             amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }

  if (cam->AmI(ifm3d::Camera::device_family::O3X) ||
      cam->AmI(ifm3d::Camera::device_family::O3R))
    {
      add<float>(amp2, 1.0f);
      EXPECT_FALSE(std::equal(amp.begin<float>(),
                              amp.end<float>(),
                              amp2.begin<float>(),
                              cmp_with_nan<float>));
    }
  else
    {
      add<uint16_t>(amp2, 1);
      EXPECT_FALSE(std::equal(amp.begin<std::uint16_t>(),
                              amp.end<std::uint16_t>(),
                              amp2.begin<std::uint16_t>()));
    }
}

TEST(StlImageBuffer, CopyAssignmentOperator)
{
  LOG(INFO) << "Image.CopyAssignmentOperator test";

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::StlImageBuffer>();
  auto im2 = std::make_shared<ifm3d::StlImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  *(im2.get()) = *(im.get());

  auto amp = im->AmplitudeImage();
  auto amp2 = im2->AmplitudeImage();

  if (cam->AmI(ifm3d::Camera::device_family::O3X) ||
      cam->AmI(ifm3d::Camera::device_family::O3R))
    {
      EXPECT_TRUE(amp.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(amp.begin<float>(),
                             amp.end<float>(),
                             amp2.begin<float>(),
                             cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(amp.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(amp.begin<std::uint16_t>(),
                             amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }

  if (cam->AmI(ifm3d::Camera::device_family::O3X) ||
      cam->AmI(ifm3d::Camera::device_family::O3R))
    {
      add<float>(amp2, 1.0f);
      EXPECT_FALSE(std::equal(amp.begin<float>(),
                              amp.end<float>(),
                              amp2.begin<float>(),
                              cmp_with_nan<float>));
    }
  else
    {
      add<uint16_t>(amp2, 1);
      EXPECT_FALSE(std::equal(amp.begin<std::uint16_t>(),
                              amp.end<std::uint16_t>(),
                              amp2.begin<std::uint16_t>()));
    }
}

TEST(StlImageBuffer, References)
{
  LOG(INFO) << "Image.References test";

  auto cam = ifm3d::Camera::MakeShared();
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam);
  auto im = std::make_shared<ifm3d::StlImageBuffer>();

  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));

  auto amp = im->AmplitudeImage();
  auto amp2 = im->AmplitudeImage();

  if (cam->AmI(ifm3d::Camera::device_family::O3X) ||
      cam->AmI(ifm3d::Camera::device_family::O3R))
    {
      EXPECT_TRUE(amp.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_32F);
      EXPECT_TRUE(amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(amp.begin<float>(),
                             amp.end<float>(),
                             amp2.begin<float>(),
                             cmp_with_nan<float>));
    }
  else
    {
      EXPECT_TRUE(amp.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(amp2.dataFormat() == ifm3d::pixel_format::FORMAT_16U);
      EXPECT_TRUE(amp.nchannels() == 1);
      EXPECT_TRUE(amp2.nchannels() == 1);
      EXPECT_TRUE(std::equal(amp.begin<std::uint16_t>(),
                             amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }

  if (cam->AmI(ifm3d::Camera::device_family::O3X) ||
      cam->AmI(ifm3d::Camera::device_family::O3R))
    {
      add<float>(amp2, 1.0f);
      EXPECT_TRUE(std::equal(amp.begin<float>(),
                             amp.end<float>(),
                             amp2.begin<float>(),
                             cmp_with_nan<float>));
    }
  else
    {
      add<uint16_t>(amp2, 1);
      EXPECT_TRUE(std::equal(amp.begin<std::uint16_t>(),
                             amp.end<std::uint16_t>(),
                             amp2.begin<std::uint16_t>()));
    }
}

TEST(StlImageBuffer, ComputeCartesian)
{
  //
  // NOTE: The conversions back/forth from/to m/mm is to
  // make this code work for all ifm3d sensors -- i.e.,
  // some return their data as float in meters, others as
  // integers in mm. We normalize to mm and do integer comparisions for
  // correctness.
  //

  auto cam = ifm3d::Camera::MakeShared();
  auto im = std::make_shared<ifm3d::StlImageBuffer>();
  //
  // 1. Stream in the unit vectors
  //
  auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_UVEC);
  ASSERT_TRUE(fg->WaitForFrame(im.get(), 1000));
  ifm3d::Image uvec = im->UnitVectors();
  //
  // 2. Now we stream in both the radial distance image and the cartesian
  // data. The latter we simply use as ground truth
  //
  fg.reset();
  fg =
    std::make_shared<ifm3d::FrameGrabber>(cam,
                                          ifm3d::IMG_RDIS | ifm3d::IMG_CART);
  EXPECT_TRUE(fg->WaitForFrame(im.get(), 1000));
  ifm3d::Image rdis = im->DistanceImage();
  ifm3d::Image conf = im->ConfidenceImage();
  ifm3d::Image xyz = im->XYZImage(); // ground truth

  std::vector<ifm3d::Image> chans(3);
  ifm3d::Image x_cam, y_cam, z_cam;
  switch (xyz.dataFormat())
    {
    case ifm3d::pixel_format::FORMAT_16U:
      split<uint16_t>(xyz, chans);
      x_cam = chans[0];
      y_cam = chans[1];
      z_cam = chans[2];
      break;
    case ifm3d::pixel_format::FORMAT_32F:
      split<float>(xyz, chans);
      mul<float>(chans[0], 1000.0f);
      mul<float>(chans[1], 1000.0f);
      mul<float>(chans[2], 1000.0f);

      // cast to int16_t
      convert_to<float, int16_t>(chans[0],
                                 x_cam,
                                 ifm3d::pixel_format::FORMAT_16S);
      convert_to<float, int16_t>(chans[1],
                                 y_cam,
                                 ifm3d::pixel_format::FORMAT_16S);
      convert_to<float, int16_t>(chans[2],
                                 z_cam,
                                 ifm3d::pixel_format::FORMAT_16S);
      break;
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
  ifm3d::Image ex, ey, ez;
  std::vector<ifm3d::Image> uvec_chans(3);
  split<float>(uvec, uvec_chans);
  ex = uvec_chans[0];
  ey = uvec_chans[1];
  ez = uvec_chans[2];

  ifm3d::Image rdis_f;
  if (cam->AmI(ifm3d::Camera::device_family::O3D))
    {
      convert_to<uint16_t, float>(rdis,
                                  rdis_f,
                                  ifm3d::pixel_format::FORMAT_32F);
    }
  if (rdis.dataFormat() == ifm3d::pixel_format::FORMAT_32F)
    {
      rdis_f = rdis;
      // assume rdis was in meters, convert to mm
      mul<float>(rdis_f, 1000.0f);
    }

  // compute
  ifm3d::Image x_ = add<float>(mul<float, float>(ex, rdis_f), tx);
  ifm3d::Image y_ = add<float>(mul<float, float>(ey, rdis_f), ty);
  ifm3d::Image z_ = add<float>(mul<float, float>(ez, rdis_f), tz);

  // blank out bad pixels ... our zero pixels will
  // be exactly equal to tx, ty, tz and if any of those
  // exceed 1cm (our test tolerance) like on an O3D301,
  // we will get errors in the unit test.
  ifm3d::Image bad_mask;

  bad_mask.create(conf.width(), conf.height(), 1, conf.dataFormat());
  int index = 0;
  auto it = bad_mask.begin<uint8_t>();
  for (uint8_t value : ifm3d::IteratorAdapter<uint8_t>(conf))
    {
      *it = value & 0x1;
      it++;
    }
  x_.setTo(0., bad_mask);
  y_.setTo(0., bad_mask);
  z_.setTo(0., bad_mask);

  //
  // 4. Cast (back) to int16 and transform to ifm3d coord frame
  //
  ifm3d::Image x_i, y_i, z_i;

  convert_to<float, int16_t>(x_, x_i, ifm3d::pixel_format::FORMAT_16S);
  convert_to<float, int16_t>(y_, y_i, ifm3d::pixel_format::FORMAT_16S);
  convert_to<float, int16_t>(z_, z_i, ifm3d::pixel_format::FORMAT_16S);

  ifm3d::Image x_computed = z_i;
  ifm3d::Image y_computed = x_i;
  ifm3d::Image z_computed = y_i;

  mul<int16_t>(y_computed, -1);
  mul<int16_t>(z_computed, -1);
  //
  // 5. Compare for correctness
  //
  auto cmp = [](std::int16_t a, std::int16_t b) -> bool {
    if (std::abs(a - b) <= 10) // 10 mm == cm accuracy
      {
        return true;
      }

    return false;
  };

  EXPECT_TRUE(std::equal(x_cam.begin<std::int16_t>(),
                         x_cam.end<std::int16_t>(),
                         x_computed.begin<std::int16_t>(),
                         cmp));

  EXPECT_TRUE(std::equal(y_cam.begin<std::int16_t>(),
                         y_cam.end<std::int16_t>(),
                         y_computed.begin<std::int16_t>(),
                         cmp));

  EXPECT_TRUE(std::equal(z_cam.begin<std::int16_t>(),
                         z_cam.end<std::int16_t>(),
                         z_computed.begin<std::int16_t>(),
                         cmp));
}

TEST(StlImageBuffer, TimeStamp)
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

  ifm3d::StlImageBuffer::Ptr img = std::make_shared<ifm3d::StlImageBuffer>();
  ifm3d::FrameGrabber::Ptr fg =
    std::make_shared<ifm3d::FrameGrabber>(cam,
                                          ifm3d::IMG_AMP | ifm3d::IMG_CART);

  std::array<ifm3d::TimePointT, 2> tps;
  // get two consecutive timestamps
  for (ifm3d::TimePointT& t : tps)
    {
      EXPECT_TRUE(fg->WaitForFrame(img.get(), 1000));
      t = img->TimeStamp();
    }
  // the first time point need to be smaller than the second one
  EXPECT_LT(tps[0], tps[1]);
  auto tdiff =
    std::chrono::duration_cast<std::chrono::milliseconds>(tps[1] - tps[0])
      .count();
  EXPECT_GT(tdiff, 20);
}

TEST(StlImageBuffer, IlluTemp)
{
  ifm3d::Camera::Ptr cam = std::make_shared<ifm3d::Camera>();

  ifm3d::StlImageBuffer::Ptr img = std::make_shared<ifm3d::StlImageBuffer>();
  ifm3d::FrameGrabber::Ptr fg = std::make_shared<ifm3d::FrameGrabber>(
    cam,
    ifm3d::DEFAULT_SCHEMA_MASK | ifm3d::ILLU_TEMP);

  ASSERT_TRUE(fg->WaitForFrame(img.get(), 1000));

  // currently not supported on O3X
  if (cam->AmI(ifm3d::Camera::device_family::O3X) ||
      cam->AmI(ifm3d::Camera::device_family::O3R))
    {
      return;
    }

  float illu_temp = img->IlluTemp();

  EXPECT_GT(illu_temp, 10);
  EXPECT_LT(illu_temp, 90);
}
