#include <cstring>
#include <gtest/gtest.h>
#include <ifm3d/common/err.h>

TEST(Err, Ok)
{
  bool ex_caught = false;

  try
    {
      throw ifm3d::Error(IFM3D_NO_ERRORS);
    }
  catch (const ifm3d::Error& ex)
    {
      ex_caught = true;
      EXPECT_EQ(IFM3D_NO_ERRORS, ex.code());
      EXPECT_STREQ("OK", ex.what());
    }
  EXPECT_TRUE(ex_caught);
}

TEST(Err, UnknownError)
{
  bool ex_caught = false;

  try
    {
      throw ifm3d::Error(0x0BCDABCD);
    }
  catch (const ifm3d::Error& ex)
    {
      ex_caught = true;
      EXPECT_EQ(0x0BCDABCD, ex.code());
      EXPECT_TRUE(strstr(ex.what(), "Unknown error") != nullptr)
        << ex.what() << " does not contain 'Unknown error'";
    }
  EXPECT_TRUE(ex_caught);
}

TEST(Err, ErrorWithMessage)
{
  bool ex_caught = false;

  try
    {
      throw ifm3d::Error(IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED,
                         "This is a test");
    }
  catch (const ifm3d::Error& ex)
    {
      ex_caught = true;
      EXPECT_EQ(IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED, ex.code());
      EXPECT_TRUE(strstr(ex.what(), "This is a test") != nullptr)
        << ex.what() << " does not contain 'This is a test'";
    }
  EXPECT_TRUE(ex_caught);
}

TEST(Err, ErrorWithoutMessage)
{
  bool ex_caught = false;

  try
    {
      throw ifm3d::Error(IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED);
    }
  catch (const ifm3d::Error& ex)
    {
      ex_caught = true;
      EXPECT_EQ(IFM3D_CONFIDENCE_IMAGE_FORMAT_NOT_SUPPORTED, ex.code());
    }
  EXPECT_TRUE(ex_caught);
}