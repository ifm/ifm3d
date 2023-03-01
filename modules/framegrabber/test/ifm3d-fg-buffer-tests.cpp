#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>
#include <ifm3d/fg/buffer.h>
#include <ifm3d/device/err.h>
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
  ifm3d::Buffer
  add(ifm3d::Buffer& img, T val)
  {
    for (auto& pix_val : ifm3d::IteratorAdapter<T>(img))
      {
        pix_val += val;
      }
    return img;
  }

} // end namespace

TEST(Buffer, Construct)
{
  ifm3d::Buffer img;

  EXPECT_TRUE(img.width() == 0);
  EXPECT_TRUE(img.height() == 0);
  EXPECT_TRUE(img.nchannels() == 0);
}

TEST(Buffer, parameter_ctor)
{
  const int height = 100;
  const int width = 100;
  const int nchannel = 1;

  ifm3d::Buffer img(width, height, 1, ifm3d::pixel_format::FORMAT_8U);

  EXPECT_TRUE(img.ptr(0) != nullptr);
  EXPECT_TRUE(img.width() == width);
  EXPECT_TRUE(img.height() == height);
  EXPECT_TRUE(img.nchannels() == nchannel);
  EXPECT_TRUE(img.dataFormat() == ifm3d::pixel_format::FORMAT_8U);
}

TEST(Buffer, copy_ctor)
{
  const int height = 100;
  const int width = 100;
  const int nchannel = 1;

  ifm3d::Buffer img1(width, height, 1, ifm3d::pixel_format::FORMAT_8U);
  ifm3d::Buffer img(img1);

  EXPECT_TRUE(img.ptr(0) == img1.ptr(0));
  EXPECT_TRUE(img.width() == img1.width());
  EXPECT_TRUE(img.height() == img1.height());
  EXPECT_TRUE(img.nchannels() == img1.nchannels());
  EXPECT_TRUE(img.dataFormat() == img1.dataFormat());

  EXPECT_TRUE(std::equal(img.begin<std::uint8_t>(),
                         img.end<std::uint8_t>(),
                         img1.begin<std::uint8_t>()));
}

TEST(Buffer, copy_assigment)
{
  const int height = 100;
  const int width = 100;
  const int nchannel = 1;

  ifm3d::Buffer img1(width, height, 1, ifm3d::pixel_format::FORMAT_8U);
  ifm3d::Buffer img = img1;

  EXPECT_TRUE(img.ptr(0) == img1.ptr(0));
  EXPECT_TRUE(img.width() == img1.width());
  EXPECT_TRUE(img.height() == img1.height());
  EXPECT_TRUE(img.nchannels() == img1.nchannels());
  EXPECT_TRUE(img.dataFormat() == img1.dataFormat());

  EXPECT_TRUE(std::equal(img.begin<std::uint8_t>(),
                         img.end<std::uint8_t>(),
                         img1.begin<std::uint8_t>()));
}

TEST(Buffer, Create)
{
  ifm3d::Buffer img;

  const int height = 100;
  const int width = 100;
  const int nchannel = 1;

  img.create(width, height, 1, ifm3d::pixel_format::FORMAT_8U);

  EXPECT_TRUE(img.ptr(0) != nullptr);
  EXPECT_TRUE(img.width() == width);
  EXPECT_TRUE(img.height() == height);
  EXPECT_TRUE(img.nchannels() == nchannel);
  EXPECT_TRUE(img.dataFormat() == ifm3d::pixel_format::FORMAT_8U);
}

// CLone

TEST(Buffer, Clone)
{
  const int height = 100;
  const int width = 100;
  const int nchannel = 1;

  ifm3d::Buffer img(width, height, 1, ifm3d::pixel_format::FORMAT_8U);
  ifm3d::Buffer img_clone = img.clone();

  EXPECT_TRUE(img.ptr(0) != img_clone.ptr(0));
  EXPECT_TRUE(img.width() == img_clone.width());
  EXPECT_TRUE(img.height() == img_clone.height());
  EXPECT_TRUE(img.nchannels() == img_clone.nchannels());
  EXPECT_TRUE(img.dataFormat() == img_clone.dataFormat());

  EXPECT_TRUE(std::equal(img.begin<std::uint8_t>(),
                         img.end<std::uint8_t>(),
                         img_clone.begin<std::uint8_t>()));
}

// accessing pointers
TEST(Buffer, row_pointer)
{
  const int height = 100;
  const int width = 100;
  const int nchannel = 1;

  ifm3d::Buffer img(width, height, 1, ifm3d::pixel_format::FORMAT_8U);

  EXPECT_TRUE(img.ptr(0) != nullptr);
  EXPECT_TRUE(img.ptr(height - 1) != nullptr);
  EXPECT_TRUE(img.ptr(10, 10) != nullptr);
}

// at function
TEST(Buffer, at)
{
  const int height = 100;
  const int width = 100;
  const int nchannel = 1;

  ifm3d::Buffer img(width, height, 1, ifm3d::pixel_format::FORMAT_16U);

  // fill the image with values
  for (int i = 0; i < height * width; i++)
    {
      EXPECT_NO_FATAL_FAILURE(img.at<uint16_t>(i) = i);
    }
  uint16_t val = 0;
  for (auto pix_val : ifm3d::IteratorAdapter<uint16_t>(img))
    {
      EXPECT_TRUE(pix_val == val);
      val++;
    }
  val = 0;
  for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col++)
        {
          EXPECT_TRUE(img.at<uint16_t>(row, col) == val);
          val++;
        }
    }
}

// SetTo
TEST(Buffer, setTo)
{
  const int height = 100;
  const int width = 100;
  const int nchannel = 1;

  ifm3d::Buffer img(width, height, 1, ifm3d::pixel_format::FORMAT_16U);
  ifm3d::Buffer mask(width, height, 1, ifm3d::pixel_format::FORMAT_8U);

  add<uint8_t>(mask, 1);
  uint16_t val = 255;

  img.setTo<uint16_t>(val, mask);
  auto mask_itr = mask.begin<uint8_t>();
  for (const auto pix_val : ifm3d::IteratorAdapter<uint16_t>(img))
    {
      if (*mask_itr == 1)
        {
          EXPECT_TRUE(pix_val == val);
        }
      mask_itr++;
    }
}

// Iterators
TEST(Buffer, iterators)
{
  const int height = 100;
  const int width = 100;
  const int nchannel = 1;

  ifm3d::Buffer img(width, height, 1, ifm3d::pixel_format::FORMAT_16U);

  // fill the image with values
  for (int i = 0; i < height * width; i++)
    {
      EXPECT_NO_FATAL_FAILURE(img.at<uint16_t>(i) = i);
    }

  auto it_begin = img.begin<uint16_t>();
  auto it_end = img.begin<uint16_t>();
  EXPECT_TRUE(it_begin != nullptr);
  EXPECT_TRUE(it_end != nullptr);
  EXPECT_TRUE(it_begin == img.ptr(0));
  uint16_t val = 0;
  auto it = it_begin;
  for (int i = 0; i < width * height; i++, val++, it++)
    {
      EXPECT_TRUE(*it == val);
    }
  val = 0;
  // check using ranged based for loop
  for (const uint16_t pix_val : ifm3d::IteratorAdapter<uint16_t>(img))
    {
      EXPECT_TRUE(pix_val == val);
      val++;
    }
}

TEST(Buffer, ptr_comparision)
{
  const int height = 100;
  const int width = 100;
  const int nchannel = 3;

  ifm3d::Buffer img(width, height, 3, ifm3d::pixel_format::FORMAT_32F);
  ifm3d::Point3D_32F point;
  // fill the image with values
  for (int i = 0; i < height * width; i++)
    {
      point.val[0] = static_cast<float>(rand());
      point.val[1] = static_cast<float>(rand());
      point.val[2] = static_cast<float>(rand());
      EXPECT_NO_FATAL_FAILURE(img.at<ifm3d::Point3D_32F>(i) = point);
    }

  for (int i = 0; i < img.height(); i++)
    {
      for (int j = 0; j < img.width(); j++)
        {
          auto ptr = img.ptr<float>(i, j);
          auto ptr_struct = img.ptr<ifm3d::Point3D_32F>(i, j);
          EXPECT_TRUE(ptr[0] == ptr_struct->val[0]);
          EXPECT_TRUE(ptr[1] == ptr_struct->val[1]);
          EXPECT_TRUE(ptr[2] == ptr_struct->val[2]);
        }
    }
}

TEST(Buffer, invalid_data_type)
{
  EXPECT_NO_THROW(ifm3d::Buffer(100, 100, 3, ifm3d::pixel_format::FORMAT_32F));
  EXPECT_THROW(
    ifm3d::Buffer img(100, 100, 3, static_cast<ifm3d::pixel_format>(1000)),
    ifm3d::Error);
}
