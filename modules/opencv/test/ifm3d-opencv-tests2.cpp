#include <ifm3d/opencv.h> // <-- DO NOT REMOVE, this is the point of this test
#include <glog/logging.h>
#include <gtest/gtest.h>

TEST(OpenCV, HeaderOnlyODR)
{
  //
  // This is a dummy. It is here simply to have the header-only library
  // included across two translation units to ensure we do not violate ODR in
  // cases where the `ifm3d/opencv.h` header is included thusly in customer
  // projects.
  //
  EXPECT_TRUE(1 == 1);
}
