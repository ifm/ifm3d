#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <ifm3d/camera.h>
#include <gtest/gtest.h>

class CameraTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    this->cam_ = ifm3d::Camera::MakeShared();
  }

  virtual void TearDown()
  {

  }

  ifm3d::Camera::Ptr cam_;
};

TEST_F(CameraTest, FactoryDefaults)
{
  EXPECT_NO_THROW(this->cam_->FactoryReset());
  std::this_thread::sleep_for(std::chrono::seconds(6));
  EXPECT_NO_THROW(this->cam_->DeviceType());
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

  // tests which only get enabled when the IFM3D_SESSION_ID
  // environment variable is properly set
  if (ifm3d::DEFAULT_SESSION_ID != "")
    {
      // default session id should be our 32 char hex string
      EXPECT_EQ(ifm3d::DEFAULT_SESSION_ID.size(), ifm3d::SESSION_ID_SZ);

      sid = this->cam_->RequestSession();
      EXPECT_STREQ(sid.c_str(), ifm3d::DEFAULT_SESSION_ID.c_str());
      EXPECT_TRUE(this->cam_->CancelSession());

      // no session open, but env var set
      EXPECT_TRUE(this->cam_->CancelSession());

      // New scope, so the `cam2` dtor runs
      {
        auto cam2 = ifm3d::Camera::MakeShared();
        sid = cam2->RequestSession();
        EXPECT_STREQ(sid.c_str(), ifm3d::DEFAULT_SESSION_ID.c_str());
        // OK, pretend `cam2` crashed ... we want to create a new session
        // but we will get an exception
        EXPECT_THROW(this->cam_->RequestSession(), ifm3d::error_t);
        // Let's now use `this->cam_` to cancel the sid from the other
        // camera
        EXPECT_TRUE(this->cam_->CancelSession(sid));
        // Now make a new session
        sid = this->cam_->RequestSession();
        EXPECT_STREQ(sid.c_str(), ifm3d::DEFAULT_SESSION_ID.c_str());
        // Now cancel the "current session on `this->cam_`
        EXPECT_TRUE(this->cam_->CancelSession());

        // once we fall off the edge of this scope, the `cam2` dtor
        // will run ... you may see a warning message in the log
        // related to the old session id trying to be closed by the
        // `cam2` dtor, this is OK
      }
    }
  else
    {
      // No session open and env var not set
      EXPECT_TRUE(this->cam_->CancelSession());

      //
      // same test as above, but w/o the comparision to some known session id
      // set via an environment var
      //
      // New scope, so the `cam2` dtor runs
      {
        auto cam2 = ifm3d::Camera::MakeShared();
        sid = cam2->RequestSession();

        // OK, pretend `cam2` crashed ... we want to create a new session
        // but we will get an exception
        EXPECT_THROW(this->cam_->RequestSession(), ifm3d::error_t);

        // Let's now use `this->cam_` to cancel the sid from the other
        // camera
        EXPECT_TRUE(this->cam_->CancelSession(sid));

        // Now make a new session
        sid = this->cam_->RequestSession();

        // Now cancel the "current session on `this->cam_`
        EXPECT_TRUE(this->cam_->CancelSession());

        // once we fall off the edge of this scope, the `cam2` dtor
        // will run ... you may see a warning message in the log
        // related to the old session id trying to be closed by the
        // `cam2` dtor, this is OK
      }
    }

  // explicitly request session ... the unit test lifecycle should
  // implicitly cancel the session for us (i.e., the cam dtor will run)
  sid = this->cam_->RequestSession();
  EXPECT_STREQ(sid.c_str(), this->cam_->SessionID().c_str());
}

TEST_F(CameraTest, CopyDeleteApplication)
{
  if (this->cam_->IsO3X())
    {
      return;
    }

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
  if (this->cam_->IsO3X())
    {
      return;
    }

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

TEST_F(CameraTest, ImportExportApplication)
{
  json app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 1);

  int idx = app_list[0]["Index"].get<int>();

  std::vector<std::uint8_t> bytes;
  int new_idx = -1;

  if (this->cam_->IsO3X())
    {
      EXPECT_NO_THROW(bytes = this->cam_->ExportIFMApp(idx));
      EXPECT_NO_THROW(new_idx = this->cam_->ImportIFMApp(bytes));
      EXPECT_EQ(new_idx, idx); // single app on O3X
    }
  else
    {
      EXPECT_NO_THROW(bytes = this->cam_->ExportIFMApp(idx));
      EXPECT_NO_THROW(new_idx = this->cam_->ImportIFMApp(bytes));

      app_list = this->cam_->ApplicationList();
      EXPECT_EQ(app_list.size(), 2);

      this->cam_->DeleteApplication(new_idx);
      app_list = this->cam_->ApplicationList();
      EXPECT_EQ(app_list.size(), 1);
    }
}

TEST_F(CameraTest, ImportExportConfig)
{
  std::vector<std::uint8_t> bytes;

  EXPECT_NO_THROW(bytes = this->cam_->ExportIFMConfig());
  EXPECT_NO_THROW(
    this->cam_->ImportIFMConfig(
      bytes,
      static_cast<std::uint16_t>(ifm3d::Camera::import_flags::GLOBAL)));
}

TEST_F(CameraTest, ActiveApplication)
{
  // factory defaults, should only be 1 application
  json app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 1);

  //
  // The rest of the test is invalid for O3X
  //
  if (this->cam_->IsO3X())
    {
      return;
    }

  // create a new application using JSON syntax
  EXPECT_NO_THROW(this->cam_->FromJSONStr(R"({"Apps":[{}]})"));

  app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 2);

  // We expect the new application to be at index 2
  int idx = -1;
  for (auto& a : app_list)
    {
      if (! a["Active"].get<bool>())
        {
          idx = a["Index"].get<int>();
          break;
        }
    }
  EXPECT_EQ(idx, 2);

  // Mark the application at index 2 as active
  EXPECT_NO_THROW(
    this->cam_->FromJSONStr(R"({"Device":{"ActiveApplication":"2"}})"));
  app_list = this->cam_->ApplicationList();
  for (auto& a : app_list)
    {
      if (a["Active"].get<bool>())
        {
          idx = a["Index"].get<int>();
          break;
        }
    }
  EXPECT_EQ(idx, 2);

  // Delete the application at index 2
  EXPECT_NO_THROW(this->cam_->DeleteApplication(idx));

  // should only have one application now
  app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 1);

  // the only application on the camera will not be active
  idx = -1;
  for (auto& a : app_list)
    {
      if (a["Active"].get<bool>())
        {
          idx = a["Index"].get<int>();
          break;
        }
    }
  EXPECT_EQ(idx, -1);

  // Mark application 1 as active
  EXPECT_NO_THROW(
    this->cam_->FromJSONStr(R"({"Device":{"ActiveApplication":"1"}})"));
  app_list = this->cam_->ApplicationList();
  for (auto& a : app_list)
    {
      if (a["Active"].get<bool>())
        {
          idx = a["Index"].get<int>();
          break;
        }
    }
  EXPECT_EQ(idx, 1);
}

TEST_F(CameraTest, ImagerTypes)
{
  json dump = this->cam_->ToJSON();
  std::string curr_im_type = dump["ifm3d"]["Apps"][0]["Imager"]["Type"];

  std::vector<std::string> im_types = this->cam_->ImagerTypes();
  json j = R"({"Apps":[]})"_json;
  for (auto& it : im_types)
    {
      dump["ifm3d"]["Apps"][0]["Imager"]["Type"] = it;
      j["Apps"] = dump["ifm3d"]["Apps"];
      this->cam_->FromJSON(j);
      dump = this->cam_->ToJSON();
      EXPECT_STREQ(
        dump["ifm3d"]["Apps"][0]["Imager"]["Type"].get<std::string>().c_str(),
        it.c_str());
    }

  dump["ifm3d"]["Apps"][0]["Imager"]["Type"] = curr_im_type;
  j["Apps"] = dump["ifm3d"]["Apps"];
  EXPECT_NO_THROW(this->cam_->FromJSON(j));
}

TEST_F(CameraTest, Filters)
{
  // factory defaults, should only be 1 application
  json app_list = this->cam_->ApplicationList();
  EXPECT_EQ(app_list.size(), 1);

  // Create a median spatial filter
  std::string j =
    R"(
        {"Apps":[{"Index":"1", "Imager": {"SpatialFilterType":"1"}}]}
      )";
  EXPECT_NO_THROW(this->cam_->FromJSONStr(j));

  // a-priori we know the median filter has a MaskSize param,
  // by default it is 3x3
  json dump = this->cam_->ToJSON();

  int mask_size = static_cast<int>(ifm3d::Camera::mfilt_mask_size::_3x3);

  if (! this->cam_->IsO3X())
    {
       mask_size =
         std::stoi(
           dump["ifm3d"]["Apps"][0]["Imager"]["SpatialFilter"]["MaskSize"].
           get<std::string>());
    }

  EXPECT_EQ(mask_size, static_cast<int>(ifm3d::Camera::mfilt_mask_size::_3x3));

  // get rid of the spatial filter
  j =
    R"(
        {"Apps":[{"Index":"1", "Imager": {"SpatialFilterType":"0"}}]}
      )";
  EXPECT_NO_THROW(this->cam_->FromJSONStr(j));

  // Create a mean temporal filter
  j =
    R"(
        {"Apps":[{"Index":"1", "Imager": {"TemporalFilterType":"1"}}]}
      )";
  EXPECT_NO_THROW(this->cam_->FromJSONStr(j));

  // a-priori we know the mean filter averages 2 images by default
  dump = this->cam_->ToJSON();
  int n_imgs = 2; // default images number

  if (! this->cam_->IsO3X())
    {
     n_imgs =
       std::stoi(
         dump["ifm3d"]["Apps"][0]["Imager"]["TemporalFilter"]["NumberOfImages"].
         get<std::string>());
    }

  EXPECT_EQ(n_imgs, 2);

  // get rid of the temporal filter
  j =
    R"(
        {"Apps":[{"Index":"1", "Imager": {"TemporalFilterType":"0"}}]}
      )";
  EXPECT_NO_THROW(this->cam_->FromJSONStr(j));
}

TEST_F(CameraTest, JSON)
{
  EXPECT_NO_THROW(this->cam_->FromJSON(this->cam_->ToJSON()));
  EXPECT_NO_THROW(this->cam_->FromJSONStr(this->cam_->ToJSONStr()));

  std::string j = R"({"Device":{"Name":"ifm3d unit test"}})";
  EXPECT_NO_THROW(this->cam_->FromJSONStr(j));
}

TEST_F(CameraTest, Time)
{
  json dump = this->cam_->ToJSON();
  if (dump["/ifm3d/Time"_json_pointer].empty())
    {
      return;
    }

  //
  // 1. check mutating the "WaitSyncTries" parameters
  //
  int n_tries =
    std::stoi(dump["ifm3d"]["Time"]["WaitSyncTries"].get<std::string>());
  int n_tries_new = n_tries == 1 ? 2 : 1;
  dump["ifm3d"]["Time"]["WaitSyncTries"] = std::to_string(n_tries_new);

  EXPECT_NO_THROW(this->cam_->FromJSON(dump));
  dump = this->cam_->ToJSON();
  n_tries =
    std::stoi(dump["ifm3d"]["Time"]["WaitSyncTries"].get<std::string>());
  EXPECT_EQ(n_tries_new, n_tries);

  //
  // 2. Make sure CurrentTime is treated as read-only
  //
  int curr_time =
    std::stoi(dump["ifm3d"]["Time"]["CurrentTime"].get<std::string>());
  dump["ifm3d"]["Time"]["CurrentTime"] = std::to_string(0);
  EXPECT_NO_THROW(this->cam_->FromJSON(dump));
  // make sure at least one full second passes
  std::this_thread::sleep_for(std::chrono::seconds(1));
  dump = this->cam_->ToJSON();
  int curr_time_new =
    std::stoi(dump["ifm3d"]["Time"]["CurrentTime"].get<std::string>());
  EXPECT_GT(curr_time_new, curr_time);

  //
  // 3. Test setting the NTP servers && activating NTP
  //
  std::string j = R"(
                       {
                         "Time":
                           {
                             "NTPServers":"91.189.89.198",
                             "SynchronizationActivated":"true"
                           }
                       }
                   )";
  EXPECT_NO_THROW(this->cam_->FromJSONStr(j));
  dump = this->cam_->ToJSON();
  std::string ip = dump["ifm3d"]["Time"]["NTPServers"].get<std::string>();
  std::string active =
    dump["ifm3d"]["Time"]["SynchronizationActivated"].get<std::string>();
  EXPECT_STREQ("91.189.89.198", ip.c_str());
  EXPECT_STREQ("true", active.c_str());

  //
  // 4. Now, undo what we did above
  //
  j = R"(
          {
            "Time":
             {
               "NTPServers":"",
               "SynchronizationActivated":"false"
             }
          }
         )";
  EXPECT_NO_THROW(this->cam_->FromJSONStr(j));
  dump = this->cam_->ToJSON();
  ip = dump["ifm3d"]["Time"]["NTPServers"].get<std::string>();
  active = dump["ifm3d"]["Time"]["SynchronizationActivated"].get<std::string>();
  EXPECT_STREQ("", ip.c_str());
  EXPECT_STREQ("false", active.c_str());

  //
  // 5. Set the time to "now"
  //
  EXPECT_NO_THROW(this->cam_->SetCurrentTime());
}

TEST_F(CameraTest, TemporaryParameters)
{
  std::unordered_map<std::string, std::string> params =
  {
    { "imager_001/ExposureTime", "6000" }
  };
  cam_->RequestSession();

  EXPECT_NO_THROW(cam_->SetTemporaryApplicationParameters(params));

  params["imager_001/ExposureTime"] = "5000";
  params["imager_001/ExposureTimeRatio"] = "40";

  EXPECT_NO_THROW(cam_->SetTemporaryApplicationParameters(params));
}
