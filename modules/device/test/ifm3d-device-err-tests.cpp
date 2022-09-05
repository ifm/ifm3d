#include <ifm3d/device.h>
#include <errno.h>
#include <gtest/gtest.h>

TEST(Err, LibraryError)
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
      EXPECT_STREQ(ifm3d::strerror(IFM3D_NO_ERRORS), ex.what());
    }
  EXPECT_TRUE(ex_caught);
}

TEST(Err, SystemError)
{
  bool ex_caught = false;

  try
    {
      throw ifm3d::Error(EAGAIN);
    }
  catch (const ifm3d::Error& ex)
    {
      ex_caught = true;
      EXPECT_EQ(EAGAIN, ex.code());
      EXPECT_STREQ(ifm3d::strerror(EAGAIN), ex.what());
    }
  EXPECT_TRUE(ex_caught);
}
