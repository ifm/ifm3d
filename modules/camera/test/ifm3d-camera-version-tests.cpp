#include <ifm3d/camera.h>
#include <gtest/gtest.h>

TEST(Version, Version)
{
  int major, minor, patch;

  ifm3d::version(&major, &minor, &patch);
  EXPECT_EQ(IFM3D_VERSION, IFM3D_MAKE_VERSION(major, minor, patch));
}
