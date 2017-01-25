#include <memory>
#include <string>
#include <vector>
#include <ifm3d/camera.h>
#include <gtest/gtest.h>

class CameraTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    this->cam_ = std::make_shared<ifm3d::Camera>();
    this->article_number_ = this->cam_->ArticleNumber();
  }

  virtual void TearDown()
  {

  }

  ifm3d::Camera::Ptr cam_;
  std::string article_number_;
};

TEST_F(CameraTest, FactoryDefaults)
{
  EXPECT_NO_THROW(this->cam_->FactoryReset());
}

TEST_F(CameraTest, DefaultCredentials)
{
  EXPECT_STREQ(this->cam_->IP().c_str(),
               ifm3d::DEFAULT_IP.c_str());
  EXPECT_EQ(this->cam_->XMLRPCPort(), ifm3d::DEFAULT_XMLRPC_PORT);
  EXPECT_EQ(this->cam_->Password(), ifm3d::DEFAULT_PASSWORD);
}

TEST_F(CameraTest, ApplicationList)
{
  json app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 1); // factory defaults, we can assume this.
  EXPECT_TRUE(app_list[0]["Active"].get<bool>());
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

TEST_F(CameraTest, CopyDeleteApplication)
{
  json app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 1);

  int idx = this->cam_->CopyApplication(1);
  app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 2);

  this->cam_->DeleteApplication(idx);
  app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 1);
}

TEST_F(CameraTest, CopyDeleteExceptions)
{
  EXPECT_THROW(this->cam_->CopyApplication(-1), ifm3d::error_t);
  EXPECT_THROW(this->cam_->DeleteApplication(-1), ifm3d::error_t);
}

TEST_F(CameraTest, CreateDeleteApplication)
{
  json app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 1);

  int idx = -1;
  std::vector<std::string> app_types = this->cam_->ApplicationTypes();
  for(auto& s : app_types)
    {
      idx = this->cam_->CreateApplication(s);
      app_list = this->cam_->ApplicationList();
      EXPECT_EQ(app_list.size(), 2);

      this->cam_->DeleteApplication(idx);
      app_list = this->cam_->ApplicationList();
      EXPECT_EQ(app_list.size(), 1);
    }
}

TEST_F(CameraTest, CreateApplicationException)
{
  EXPECT_THROW(this->cam_->CreateApplication("Foo"),
               ifm3d::error_t);
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
  EXPECT_EQ(1,1);
}
