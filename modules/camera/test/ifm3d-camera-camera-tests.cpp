#include <memory>
#include <ifm3d/camera.h>
#include <gtest/gtest.h>

class CameraTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    cam_ = std::make_shared<ifm3d::Camera>();
  }

  virtual void TearDown()
  {

  }

  ifm3d::Camera::Ptr cam_;
};

TEST_F(CameraTest, DefaultCredentials)
{
  EXPECT_STREQ(this->cam_->IP().c_str(),
               ifm3d::DEFAULT_IP.c_str());
  EXPECT_EQ(this->cam_->XMLRPCPort(), ifm3d::DEFAULT_XMLRPC_PORT);
  EXPECT_EQ(this->cam_->Password(), ifm3d::DEFAULT_PASSWORD);
}

TEST_F(CameraTest, ToJSON)
{
  std::cout << this->cam_->ToJSONStr() << std::endl;
  EXPECT_EQ(1,1);
}
