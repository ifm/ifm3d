#include <memory>
#include <string>
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

TEST_F(CameraTest, SessionManagement)
{
  // explicitly request/cancel a session
  std::string sid = this->cam_->RequestSession();
  EXPECT_STREQ(sid.c_str(), this->cam_->SessionID().c_str());
  bool retval = this->cam_->CancelSession();
  EXPECT_TRUE(retval);
  EXPECT_STREQ("", this->cam_->SessionID().c_str());

  // explicitly create but implicitly cancel (via dtor) session
  sid = this->cam_->RequestSession();
  EXPECT_STREQ(sid.c_str(), this->cam_->SessionID().c_str());
  this->cam_.reset(new ifm3d::Camera());
  EXPECT_STREQ("", this->cam_->SessionID().c_str());

  // explicitly request session ... the unit test lifecycle should
  // implicitly cancel the session for us (i.e., the cam dtor will run)
  sid = this->cam_->RequestSession();
  EXPECT_STREQ(sid.c_str(), this->cam_->SessionID().c_str());
}

TEST_F(CameraTest, ApplicationList)
{
  //
  // Figure out a reasonable test for this
  //
  json app_list = this->cam_->ApplicationList();
  std::cout << app_list.dump(2) << std::endl;
}

TEST_F(CameraTest, ActiveApplication)
{
  //
  // Create a new application, mark it as active
  //
  EXPECT_EQ(1,1);
}

TEST_F(CameraTest, ToJSON)
{
  //
  // Figure out a reasonable test for this
  //
  std::cout << this->cam_->ToJSONStr() << std::endl;
  std::cout << this->cam_->ArticleNumber() << std::endl;
  EXPECT_EQ(1,1);
}
